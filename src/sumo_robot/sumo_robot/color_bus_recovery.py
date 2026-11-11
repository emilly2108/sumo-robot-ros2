"""Restricted local I2C recovery helper for the operational Pi5 robot.

ROS stays unprivileged. A root-owned installed copy serves fixed I2C1
recovery requests; no caller-supplied commands, GPIOs, addresses or paths.
"""
import argparse
import ctypes
import ctypes.util
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pwd
import signal
import socket
import stat
import struct
import subprocess
import sys
import time

RUNTIME = Path('/run/sumo-color-i2c')
SOCKET = RUNTIME/'recovery.sock'
DEVICE = Path('/sys/devices/platform/axi/1000120000.pcie/1f00074000.i2c')
DRIVER = Path('/sys/bus/platform/drivers/i2c_designware')
DEVNODE = Path('/dev/i2c-1')


def driver_ready():
    link=DEVICE/'driver'
    return link.exists() and link.resolve()==DRIVER and DEVNODE.exists()


def restore_driver():
    if not DEVICE.exists():return False
    link=DEVICE/'driver'
    if not link.exists(): (DRIVER/'bind').write_text(DEVICE.name)
    elif link.resolve()!=DRIVER:return False
    end=time.monotonic()+.5
    while not DEVNODE.exists() and time.monotonic()<end:time.sleep(.01)
    return driver_ready()


def safe_restore_driver():
    try:return restore_driver()
    except OSError as exc:
        print('I2C_DRIVER_RESTORE_FAILED',str(exc),flush=True)
        return False


def request_recovery(action='recover',socket_path=str(SOCKET),timeout=4.0):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(timeout);client.connect(socket_path)
        client.sendall((json.dumps({'action':action})+'\n').encode())
        data=b''
        while b'\n' not in data:
            chunk=client.recv(4096)
            if not chunk:raise ConnectionError('I2C helper closed without a result')
            data+=chunk
            if len(data)>65536:raise ValueError('I2C helper result too large')
    return json.loads(data.split(b'\n',1)[0])


def assert_bus_idle():
    # The robot service stays active while this helper repairs its color bus.
    # Ownership is checked using color processes and open I2C descriptors.
    for p in Path('/proc').glob('[0-9]*'):
        try:argv=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
        except OSError:continue
        actual=argv[:2]
        if (any(x.endswith(('/sumo_robot/color_node','/color_fault_fixture.py')) for x in actual)
            or any(x=='-m' and i+1<len(argv) and argv[i+1]=='sumo_robot.color_node' for i,x in enumerate(argv))):
            raise RuntimeError('Color process still exists; refusing shared bus reset')
        try:files=list((p/'fd').iterdir())
        except OSError:continue
        for fd in files:
            try:target=os.readlink(fd)
            except OSError:continue
            if target==str(DEVNODE):raise RuntimeError('I2C1 descriptor is still open in PID '+p.name)


def recover_worker(exercise=False):
    result={'ok':False,'clock_pulses':0,'exercise':exercise,'motor_commands_sent':False}
    started=time.monotonic();chip=None;requested=[];lib=None;detached=False
    def interrupted(signum,frame):raise KeyboardInterrupt('Recovery worker interrupted')
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    try:
        if os.geteuid()!=0:raise PermissionError('I2C helper worker requires root')
        assert_bus_idle()
        result['preflight_passed']=True
        if not driver_ready():raise RuntimeError('Unexpected or unbound I2C1 driver')
        if Path('/sys/class/i2c-adapter/i2c-1').resolve().parent!=DEVICE:raise RuntimeError('Unexpected I2C adapter')
        lib=ctypes.CDLL(ctypes.util.find_library('gpiod'),use_errno=True)
        signatures={
            'gpiod_chip_open_by_name':([ctypes.c_char_p],ctypes.c_void_p),
            'gpiod_chip_label':([ctypes.c_void_p],ctypes.c_char_p),
            'gpiod_chip_get_line':([ctypes.c_void_p,ctypes.c_uint],ctypes.c_void_p),
            'gpiod_line_name':([ctypes.c_void_p],ctypes.c_char_p),
            'gpiod_line_request_output_flags':([ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int,ctypes.c_int],ctypes.c_int),
            'gpiod_line_get_value':([ctypes.c_void_p],ctypes.c_int),
            'gpiod_line_set_value':([ctypes.c_void_p,ctypes.c_int],ctypes.c_int),
            'gpiod_line_release':([ctypes.c_void_p],None),
            'gpiod_chip_close':([ctypes.c_void_p],None),
        }
        for name,(args,ret) in signatures.items():
            fn=getattr(lib,name);fn.argtypes=args;fn.restype=ret
        chip=lib.gpiod_chip_open_by_name(b'gpiochip4')
        if not chip or lib.gpiod_chip_label(chip)!=b'pinctrl-rp1':raise RuntimeError('Unexpected GPIO chip')
        sda,scl=[lib.gpiod_chip_get_line(chip,n) for n in (2,3)]
        if not sda or not scl or lib.gpiod_line_name(sda)!=b'GPIO2' or lib.gpiod_line_name(scl)!=b'GPIO3':raise RuntimeError('Unexpected I2C pin names')
        (DRIVER/'unbind').write_text(DEVICE.name);detached=True
        result['controller_unbound']=True
        for line in (sda,scl):
            # libgpiod v1 OPEN_DRAIN flag. 1 releases the line, never drives HIGH.
            if lib.gpiod_line_request_output_flags(line,b'sumo-color-i2c-recovery',1,1):
                raise OSError(ctypes.get_errno(),'Cannot request open-drain I2C pin')
            requested.append(line)
        def value(line):
            v=lib.gpiod_line_get_value(line)
            if v<0:raise OSError(ctypes.get_errno(),'GPIO read failed')
            return v
        def set_value(line,v):
            if lib.gpiod_line_set_value(line,v):raise OSError(ctypes.get_errno(),'GPIO write failed')
        def high():
            set_value(scl,1);end=time.monotonic()+.05
            while value(scl)==0 and time.monotonic()<end:time.sleep(.0001)
            return value(scl)==1
        time.sleep(.01)
        result['initial_lines']={'sda':value(sda),'scl':value(scl)}
        if not high():raise RuntimeError('SCL stays LOW; software recovery cannot release it')
        for i in range(9):
            if value(sda) and not exercise:break
            set_value(scl,0);time.sleep(.00005)
            if not high():raise RuntimeError('SCL did not release during bus clear')
            time.sleep(.00005);result['clock_pulses']=i+1
        set_value(scl,0);set_value(sda,0);time.sleep(.00005)
        if not high():raise RuntimeError('Cannot generate STOP while SCL is LOW')
        time.sleep(.00005);set_value(sda,1);time.sleep(.00005)
        result['stop_generated']=True
        result['final_lines']={'sda':value(sda),'scl':value(scl)}
        if result['final_lines']!={'sda':1,'scl':1}:raise RuntimeError('I2C line remains held LOW')
    except BaseException as exc:result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        # Finish cleanup even after the first termination request.
        signal.signal(signal.SIGTERM,signal.SIG_IGN);signal.signal(signal.SIGINT,signal.SIG_IGN)
        if lib is not None:
            for line in reversed(requested):
                lib.gpiod_line_set_value(line,1);lib.gpiod_line_release(line)
            if chip:lib.gpiod_chip_close(chip)
        if detached:
            try:result['driver_restored']=restore_driver()
            except BaseException as exc:result['restore_error']=str(exc);result['driver_restored']=False
        else:result['driver_restored']=driver_ready()
    if result.get('driver_restored') and 'error' not in result:
        fd=None
        try:
            fd=os.open(DEVNODE,os.O_RDWR);fcntl.ioctl(fd,0x0703,0x70)
            os.write(fd,b'\x00');result['mux_mask']=os.read(fd,1).hex()
            result['mux_responding']=result['mux_mask']=='00';result['ok']=result['mux_responding']
        except OSError as exc:result['error']='TCA probe failed: '+str(exc)
        finally:
            if fd is not None:os.close(fd)
    result['elapsed_s']=round(time.monotonic()-started,4)
    return result


def serve():
    if os.geteuid()!=0:raise PermissionError('Recovery server requires root')
    directory=RUNTIME.lstat()
    if not stat.S_ISDIR(directory.st_mode) or directory.st_uid!=0 or directory.st_mode&0o022:
        raise RuntimeError('Recovery runtime directory must be root owned and not writable by others')
    lock=(RUNTIME/'server.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    owner=pwd.getpwnam('mars');source=Path(__file__).resolve()
    source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    running=True;worker=None;sequence=0
    def stop(signum,frame):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    if SOCKET.exists():SOCKET.unlink()
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(str(SOCKET))
    os.chown(SOCKET,owner.pw_uid,owner.pw_gid);os.chmod(SOCKET,0o600)
    server.listen(4);server.settimeout(.2)
    (RUNTIME/'server.pid').write_text(str(os.getpid())+'\n')
    print('COLOR_I2C_HELPER_READY',str(SOCKET),source_hash,flush=True)
    notify=os.environ.get('NOTIFY_SOCKET')
    if notify:
        address='\0'+notify[1:] if notify.startswith('@') else notify
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as ready:
            ready.connect(address);ready.sendall(b'READY=1\nSTATUS=Restricted I2C recovery socket ready')
    try:
        while running:
            if worker is not None and worker.poll() is not None:
                worker.communicate();worker=None
                safe_restore_driver()
            try:conn,_=server.accept()
            except socket.timeout:continue
            with conn:
                conn.settimeout(.5)
                try:
                    pid,uid,gid=struct.unpack('3i',conn.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
                    if uid not in (0,owner.pw_uid):raise PermissionError('Unauthorized helper peer')
                    raw=b''
                    while b'\n' not in raw:
                        chunk=conn.recv(1024)
                        if not chunk:raise ValueError('Empty helper request')
                        raw+=chunk
                        if len(raw)>2048:raise ValueError('Helper request too large')
                    query=json.loads(raw.split(b'\n',1)[0]);action=query.get('action')
                    if action=='status':
                        result={'ok':True,'driver_restored':driver_ready(),'busy':worker is not None,
                                'source_hash':source_hash,'pid':os.getpid(),'sequence':sequence}
                    elif action=='shutdown':running=False;result={'ok':True,'shutdown':True}
                    elif action in ('recover','exercise'):
                        if worker is not None:raise RuntimeError('Previous recovery worker has not exited')
                        sequence+=1;start=time.monotonic()
                        command=[sys.executable,'-I',str(source),'worker']
                        if action=='exercise':command.append('--exercise')
                        worker=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
                        timed_out=False
                        try:out,err=worker.communicate(timeout=2.5)
                        except subprocess.TimeoutExpired:
                            timed_out=True;worker.terminate()
                            try:out,err=worker.communicate(timeout=.5)
                            except subprocess.TimeoutExpired:
                                worker.kill()
                                try:out,err=worker.communicate(timeout=.5)
                                except subprocess.TimeoutExpired:out='';err='Worker cannot exit yet'
                        exited=worker.poll() is not None
                        result=json.loads(out.strip().splitlines()[-1]) if out.strip() else {'ok':False,'error':err[-1000:]}
                        if timed_out:result.update(ok=False,worker_timed_out=True)
                        if exited:
                            # GPIO descriptors close on worker death. Restore the
                            # fixed controller here even if its finally was killed.
                            restored=safe_restore_driver();worker=None
                        else:restored=False
                        result.update(worker_exited=exited,driver_restored=restored,
                                      safe_to_start=exited and restored and result.get('preflight_passed',False),sequence=sequence,
                                      elapsed_s=round(time.monotonic()-start,4))
                        with (RUNTIME/'events.jsonl').open('a') as log:log.write(json.dumps(result)+'\n')
                    else:raise ValueError('Only status/recover/exercise/shutdown are supported')
                except Exception as exc:result={'ok':False,'error':f'{type(exc).__name__}: {exc}','safe_to_start':False}
                try:conn.sendall((json.dumps(result)+'\n').encode())
                except (BrokenPipeError,ConnectionResetError):pass
    finally:
        server.close()
        if worker is not None:
            worker.terminate()
            try:worker.communicate(timeout=.5)
            except subprocess.TimeoutExpired:
                worker.kill()
                try:worker.communicate(timeout=.5)
                except subprocess.TimeoutExpired:pass
            if worker.poll() is not None:safe_restore_driver()
        if SOCKET.exists():SOCKET.unlink()
        lock.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['server','worker','status','recover','exercise','shutdown'])
    parser.add_argument('--exercise',action='store_true')
    args=parser.parse_args()
    if args.action=='server':serve()
    elif args.action=='worker':print(json.dumps(recover_worker(args.exercise)),flush=True)
    else:print(json.dumps(request_recovery(args.action)),flush=True)


if __name__=='__main__':main()
