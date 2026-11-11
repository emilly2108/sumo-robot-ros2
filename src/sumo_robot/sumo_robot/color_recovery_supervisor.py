"""Color process supervision for the operational robot workspace."""
import argparse
import json
import os
from pathlib import Path
import pwd
import signal
import subprocess
import sys
import time

from .color_bus_recovery import request_recovery, driver_ready, restore_driver


def is_color_process(argv):
    """Match the executed program, not paths supplied as supervisor options."""
    scripts=argv[:2]
    return (any(a.endswith('/sumo_robot/color_node') or a.endswith('/color_fault_fixture.py')
                for a in scripts)
            or any(a=='-m' and i+1<len(argv) and argv[i+1]=='sumo_robot.color_node'
                   for i,a in enumerate(argv)))


def service_conflicts():
    """Allow our own systemd service, reject a second manual controller."""
    try:
        result=subprocess.run(['systemctl','show','sumo-robot.service','--property=MainPID','--value'],
                              capture_output=True,text=True,timeout=.5)
        main_pid=int(result.stdout.strip() or '0')
    except (OSError,ValueError,subprocess.TimeoutExpired):return False
    if main_pid<=0:return False
    pid=os.getpid();visited=set()
    while pid>0 and pid not in visited:
        if pid==main_pid:return False
        visited.add(pid)
        try:
            status=(Path('/proc')/str(pid)/'status').read_text().splitlines()
            pid=int(next(x.split()[1] for x in status if x.startswith('PPid:')))
        except (OSError,ValueError,StopIteration):break
    return True


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--record-dir',required=True)
    parser.add_argument('--topic',default='/color_sensor')
    parser.add_argument('--node-name',default='color_node')
    parser.add_argument('--restart-delay',type=float,default=0.1)
    parser.add_argument('--stop-grace-seconds',type=float,default=0.5)
    parser.add_argument('--stale-reset-seconds',type=float,default=3.0)
    parser.add_argument('--hang-seconds',type=float,default=2.0)
    parser.add_argument('--enable-bus-recovery',action='store_true')
    parser.add_argument('--bus-helper')
    parser.add_argument('--bus-recovery-socket')
    parser.add_argument('--child-runner')
    parser.add_argument('--preflight-only',action='store_true')
    args=parser.parse_args()
    record=Path(args.record_dir);record.mkdir(parents=True,exist_ok=True)
    workspace=Path('/home/mars/sumo_ws')
    if not Path(__file__).resolve().is_relative_to(workspace):
        raise RuntimeError('Production supervisor must load from /home/mars/sumo_ws')
    if args.enable_bus_recovery and not args.bus_recovery_socket and (os.geteuid()!=0 or not args.bus_helper):
        raise RuntimeError('Bus recovery requires the prepared privileged helper.')
    if service_conflicts():
        raise RuntimeError('Another robot service already owns the sensor bus.')
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:argv=[a.decode(errors='replace') for a in p.read_bytes().split(b'\0') if a]
        except OSError:continue
        if p.parent.name!=str(os.getpid()) and is_color_process(argv):
            raise RuntimeError('Another color process is using the shared sensor bus.')
    if args.preflight_only:
        print('SUPERVISOR_PREFLIGHT_PASSED',str(workspace),flush=True)
        return
    running=True
    def stop(signum,frame):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    owner=pwd.getpwnam('mars')
    event_log=(record/'events.jsonl').open('a',buffering=1)
    def event(kind,**values):
        data={'event':kind,'mono_ns':time.monotonic_ns(),**values}
        event_log.write(json.dumps(data)+'\n');print('COLOR_SUPERVISOR',json.dumps(data),flush=True)
    status=record/'child_status.json'
    child=None;child_started=0;generation=0;last_production_check=0
    next_start=0;bus_recovery_due=False;last_milestone=None
    if os.geteuid()==0:
        os.chown(record,owner.pw_uid,owner.pw_gid)
        os.chown(record/'events.jsonl',owner.pw_uid,owner.pw_gid)
    def stop_child(reason):
        nonlocal child
        if child is None:return
        stop_started=time.monotonic()
        phase='already_exited'
        if child.poll() is None:
            phase='sigint'
            # A stopped process must be continued to handle SIGINT cleanup.
            try:child.send_signal(signal.SIGCONT);child.send_signal(signal.SIGINT)
            except ProcessLookupError:pass
            normal_stop=reason=='supervisor_stop'
            try:child.wait(timeout=3 if normal_stop else args.stop_grace_seconds)
            except subprocess.TimeoutExpired:
                phase='sigterm'
                child.terminate()
                try:child.wait(timeout=2 if normal_stop else 0.5)
                except subprocess.TimeoutExpired:
                    phase='sigkill'
                    child.kill()
                    try:child.wait(timeout=2 if normal_stop else 0.5)
                    except subprocess.TimeoutExpired:
                        event('child_stop_pending',pid=child.pid,reason=reason)
                        return False
        event('child_stopped',pid=child.pid,reason=reason,returncode=child.returncode,
              phase=phase,elapsed_s=round(time.monotonic()-stop_started,4))
        child=None
        return True
    try:
        event('supervisor_started',pid=os.getpid(),workspace=str(workspace),bus_recovery=args.enable_bus_recovery,
              hang_seconds=args.hang_seconds,stale_reset_seconds=args.stale_reset_seconds,
              restart_delay=args.restart_delay,stop_grace_seconds=args.stop_grace_seconds,
              node_name=args.node_name,topic=args.topic,
              discovery_profile=os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'))
        if args.enable_bus_recovery and args.bus_recovery_socket:
            try:event('bus_helper_status',**request_recovery('status',args.bus_recovery_socket,1.0))
            except (OSError,ValueError) as exc:event('bus_helper_unavailable',error=str(exc))
        while running:
            now=time.monotonic()
            if now-last_production_check>=1:
                last_production_check=now
                if service_conflicts():
                    event('stop_for_other_robot_service');break
            if child is None:
                if now<next_start:time.sleep(0.05);continue
                if bus_recovery_due:
                    bus_recovery_due=False
                    if args.enable_bus_recovery:
                        event('bus_recovery_started')
                        try:
                            if args.bus_recovery_socket:
                                result=request_recovery('recover',args.bus_recovery_socket,4.5)
                                (record/'bus_clear_result.json').write_text(json.dumps(result,indent=2)+'\n')
                            else:
                                # Compatibility for old standalone root trials.
                                # Terminate cooperatively before any forced kill,
                                # then restore the fixed driver from this parent.
                                env=os.environ.copy();env['SUMO_COLOR_RECOVERY_RECORD_DIR']=str(record)
                                p=subprocess.Popen([sys.executable,args.bus_helper],env=env)
                                try:p.wait(timeout=2.5)
                                except subprocess.TimeoutExpired:
                                    p.terminate()
                                    try:p.wait(timeout=.5)
                                    except subprocess.TimeoutExpired:p.kill();p.wait(timeout=.5)
                                restored=restore_driver()
                                result=json.loads((record/'bus_clear_result.json').read_text())
                                result.update(ok=p.returncode==0 and result.get('software_bus_clear_succeeded',False),
                                              safe_to_start=restored,driver_restored=restored)
                            event('bus_recovery_finished',returncode=0 if result.get('ok') else 1,
                                  responding=result.get('ok',False),result=result)
                            if not result.get('safe_to_start',False):
                                bus_recovery_due=True;next_start=time.monotonic()+.25
                                event('bus_recovery_wait',reason=result.get('error','Worker or I2C driver not ready'))
                                continue
                        except (OSError,ValueError,subprocess.TimeoutExpired) as exc:
                            event('bus_recovery_failed',error=str(exc))
                            # A timed-out request may still own the pins. Do not
                            # start a new color process until ownership is known.
                            if not isinstance(exc,(FileNotFoundError,ConnectionRefusedError)) or not driver_ready():
                                bus_recovery_due=True;next_start=time.monotonic()+.25
                                continue
                    else:event('bus_recovery_not_enabled')
                    if not running:break
                env=os.environ.copy();env['SUMO_COLOR_STATUS_FILE']=str(status)
                command=[sys.executable]
                if args.child_runner:command.append(args.child_runner)
                else:command+=['-m','sumo_robot.color_node']
                command+=['--ros-args','-r','__node:='+args.node_name,'-r','/color_sensor:='+args.topic]
                credentials={}
                if os.geteuid()==0:
                    credentials={'user':owner.pw_uid,'group':owner.pw_gid,
                                 'extra_groups':os.getgrouplist('mars',owner.pw_gid)}
                    env.update(HOME=owner.pw_dir,USER='mars',LOGNAME='mars')
                child=subprocess.Popen(command,env=env,**credentials)
                child_started=time.monotonic();generation+=1
                event('child_started',pid=child.pid,generation=generation)
            if child.poll() is not None:
                event('child_exited',pid=child.pid,returncode=child.returncode,generation=generation)
                child=None;bus_recovery_due=args.enable_bus_recovery
                next_start=time.monotonic()+args.restart_delay
                continue
            try:
                data=json.loads(status.read_text())
                if data.get('pid')!=child.pid:raise ValueError('Previous child status')
            except (OSError,ValueError):
                data={}
            now_ns=time.monotonic_ns()
            milestone=(child.pid,data.get('first_valid_published_ns',0))
            if milestone[1] and milestone!=last_milestone:
                last_milestone=milestone
                event('child_first_valid',pid=child.pid,generation=generation,
                      child_started_ns=int(child_started*1e9),
                      node_created_ns=data.get('node_created_ns'),
                      initialization_started_ns=data.get('initialization_started_ns'),
                      initialized_ns=data.get('initialized_ns'),
                      first_valid_published_ns=milestone[1])
            progress=data.get('progress_ns',int(child_started*1e9))
            latest_valid=data.get('last_valid_ns',0)
            hung=(now_ns-progress)/1e9>args.hang_seconds
            no_fresh=(now_ns-(latest_valid or int(child_started*1e9)))/1e9>args.stale_reset_seconds
            if hung or no_fresh:
                reason='progress_stalled' if hung else 'fresh_data_stalled'
                event('recovery_escalation',reason=reason,pid=child.pid,state=data.get('state'),
                      progress_age_s=round((now_ns-progress)/1e9,4),
                      fresh_age_s=round((now_ns-(latest_valid or int(child_started*1e9)))/1e9,4),
                      last_valid_ns=latest_valid,stage=data.get('stage'))
                if stop_child(reason) is False:
                    time.sleep(.05)
                    continue
                bus_recovery_due=True;next_start=time.monotonic()+args.restart_delay
                continue
            time.sleep(0.05)
    finally:
        stop_child('supervisor_stop')
        event('supervisor_stopped')
        event_log.close()


if __name__=='__main__':main()
