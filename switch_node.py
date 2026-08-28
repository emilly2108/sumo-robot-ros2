#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from gpiozero import Button

class SwitchNode(Node):
    def __init__(self):
        super().__init__('switch_node')
        
        # 1. 기존 brain_node의 'switch_mode' 서비스와 연결할 클라이언트 생성
        self.client = self.create_client(SetBool, 'switch_mode')
        
        # 서비스 서버가 켜질 때까지 터미널을 블로킹하며 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ brain_node의 [switch_mode] 서비스 서버가 켜지기를 대기 중...')
            
        # 2. 라즈베리파이 5 토글 스위치 설정 (GPIO 17번, 내부 풀업 자동 활성화)
        self.toggle_switch = Button(17, bounce_time=0.05)
        
        # 상태 관리를 위한 제어 변수
        self.current_physical_state = False  # 현재 스위치의 물리적 위치
        self.last_confirmed_state = None    # 제어 노드(서버)가 최종 승인한 모드 상태
        self.is_waiting_response = False     # 현재 서비스 응답을 기다리는 중인지 여부
        
        # 3. 0.1초(10Hz) 주기로 스위치 상태를 감시하고 동기화하는 타이머 가동
        self.create_timer(0.1, self.check_and_sync_switch)
        self.get_logger().info('🚀 스위치 동기화 노드가 시작되었습니다. 토글 스위치를 제어하세요.')

    def check_and_sync_switch(self):
        # gpiozero 규칙: 스위치가 켜지면(GND와 연결되면) True, 꺼지면 False
        is_on = self.toggle_switch.is_pressed
        
        # 이미 전송 후 답변을 기다리는 중이라면 중복 호출 방지
        if self.is_waiting_response:
            return
            
        # ★ [핵심 로직] 현재 물리 스위치 상태가 '제어 노드가 승인한 상태'와 다르다면?
        # 처음 켰을 때(None 일 때)나, 통신 실패로 아직 확인을 못 받았을 때 성공할 때까지 계속 진입합니다.
        if is_on != self.last_confirmed_state:
            self.send_mode_request(is_on)

    def send_mode_request(self, state):
        self.is_waiting_response = True
        self.current_physical_state = state
        
        req = SetBool.Request()
        req.data = state
        
        self.get_logger().info(f'🔄 [요청] 제어 노드로 모드 변경 시도... (목표: {state})')
        
        # 비동기 방식으로 서비스 호출 (노드가 멈추는 현상 방지)
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ [확인 완료] 제어 노드가 신호를 접수했습니다: {response.message}')
                # 제어 노드가 승인했으므로 최종 확인 상태를 업데이트 (이제 재시도를 멈춥니다)
                self.last_confirmed_state = self.current_physical_state
            else:
                self.get_logger().error(f'❌ [거부] 제어 노드가 요청을 거부함: {response.message}')
        except Exception as e:
            self.get_logger().error(f'🚨 [통신 실패] brain_node 연결 오류 (다음 루프에서 즉시 재시도): {e}')
        
        # 대기 플래그를 해제하여 실패 시 다음 0.1초 타이머 루프에서 다시 쏠 수 있도록 개방
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
