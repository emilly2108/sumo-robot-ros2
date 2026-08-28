# -*- coding: utf-8 -*-
"""기능별로 분리된 HFSM Brain 노드를 실행하는 시작 파일이다.

기존 test_code.py와 test_code_hfsm.py는 수정하지 않고 이 파일을 따로 실행한다.
"""

# 실제 ROS2 노드 생성과 spin 처리가 들어 있는 패키지의 main 함수를 가져온다.
from hfsm_brain.node import main


# 이 파일을 python3로 직접 실행했을 때만 ROS2 Brain 노드를 시작한다.
if __name__ == "__main__":
    # rclpy 초기화, 노드 실행, 종료 정리를 담당하는 main 함수를 호출한다.
    main()
