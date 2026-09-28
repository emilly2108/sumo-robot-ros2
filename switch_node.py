#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from gpiozero import Button

class SwitchNode(Node):
    def __init__(self):
        super().__init__('switch_node')

        self.client = self.create_client(SetBool, 'switch_mode')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' brain_node의 [switch_mode] 서비스 서버가 켜지기를 대기 중...')

        self.toggle_switch = Button(17, bounce_time=0.05)

        self.current_physical_state = False
        self.last_confirmed_state = None
        self.is_waiting_response = False

        self.create_timer(0.1, self.check_and_sync_switch)
        self.get_logger().info(' 스위치 동기화 노드가 시작되었습니다. 토글 스위치를 제어하세요.')

    def check_and_sync_switch(self):
        is_on = True

        if self.is_waiting_response:
            return

        if is_on != self.last_confirmed_state:
            self.send_mode_request(is_on)

    def send_mode_request(self, state):
        self.is_waiting_response = True
        self.current_physical_state = state

        req = SetBool.Request()
        req.data = state

        self.get_logger().info(f'🔄 [요청] 제어 노드로 모드 변경 시도... (목표: {state})')

        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ [확인 완료] 제어 노드가 신호를 접수했습니다: {response.message}')
                self.last_confirmed_state = self.current_physical_state
            else:
                self.get_logger().error(f'❌ [거부] 제어 노드가 요청을 거부함: {response.message}')
        except Exception as e:
            self.get_logger().error(f'🚨 [통신 실패] brain_node 연결 오류 (다음 루프에서 즉시 재시도): {e}')

        self.is_waiting_response = False

def main(args=None):
    rclpy.init(args=args)
    node = SwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
          rclpy.shutdown()

if __name__ == '__main__':
    main()
