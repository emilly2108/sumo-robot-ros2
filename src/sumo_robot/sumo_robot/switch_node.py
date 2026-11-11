#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from gpiozero import Button

class SwitchNode(Node):
    def __init__(self):
        super().__init__('switch_node')
        
        self.client = self.create_client(SetBool, 'switch_mode')
        
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                ' hfsm_brain_node의 [switch_mode] 서비스 서버가 켜지기를 대기 중...'
            )
            
        self.fight_input = Button(27, pull_up = True, bounce_time = 0.05)
        self.ready_input = Button(23, pull_up = True, bounce_time = 0.05)
        
        self.current_physical_state = False  
        self.last_confirmed_state = None    
        self.is_waiting_response = False
        self.invalid_state_reported = False 
        
        self.create_timer(0.1, self.check_and_sync_switch)
        self.get_logger().info(' 스위치 동기화 노드가 시작되었습니다. 토글 스위치를 제어하세요.')

    def check_and_sync_switch(self):
        fight_selected = self.fight_input.is_pressed
        ready_selected = self.ready_input.is_pressed

        # 정상적인 ON/ON 스위치는 둘 중 하나만 GND와 연결되어야 한다.
        if fight_selected == ready_selected:
            if not self.invalid_state_reported:
                if fight_selected:
                    self.get_logger().error('스위치 2 오류: GPIO27과 GPIO23이 모두 LOW입니다.')

                else:
                    self.get_logger().warning("스위치 2 입력 없음 : 전환 중이거나 배선을 확인하세요.")

                self.invalid_state_reported = True
            return

        self.invalid_state_reported = False

        # GPIO27 선택 → FIGHT(True)
        # GPIO23 선택 → READY(False)
        is_fight = fight_selected

        if self.is_waiting_response:
            return

        # 처음 실행했거나 물리 스위치 상태가 바뀐 경우에만 요청한다.
        if is_fight != self.last_confirmed_state:
            mode_name = "FIGHT" if is_fight else "READY"
            self.get_logger().info(f"스위치 2 입력: {mode_name}")
            self.send_mode_request(is_fight)


    def send_mode_request(self, state):
        self.is_waiting_response = True
        self.current_physical_state = state
        
        request = SetBool.Request()
        request.data = state

        mode_name = "FIGHT" if state else "READY"
        self.get_logger().info(f" [요청] 브레인 모드 변경: {mode_name}")
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()

            if response.success:
                self.last_confirmed_state = self.current_physical_state
                mode_name = ("FIGHT" if self.last_confirmed_state else "READY")

                self.get_logger().info(f"[확인 완료] 브레인 모드: {mode_name} - {response.message}")
            
            else:
                self.get_logger().error(f"[거부] 제어 노드가 요청을 거부함: {response.message}")
        
        except Exception as error:
            self.get_logger().error(
                "hfsm_brain_node 연결 오류 " f"(다음 주기에서 재시도): {error}"
            )

        finally:
            self.is_waiting_response = False

    def destroy_node(self):
        self.fight_input.close()
        self.ready_input.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SwitchNode()

        if rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
