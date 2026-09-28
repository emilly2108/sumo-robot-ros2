import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from center_calibration import calibrated_center_x, load_calibration
from tilt_detector import TiltDetector


class GreenTrackerDepth(Node):
    def __init__(self):
        super().__init__('green_tracker_depth')

        self.pub_error = self.create_publisher(Float32, '/target_error', 10)
        self.pub_found = self.create_publisher(Bool, '/target_found', 10)
        self.pub_distance = self.create_publisher(Float32, '/target_distance', 10)
        self.pub_wall_distance = self.create_publisher(Float32, '/wall_distance', 10)
        self.pub_wall_left = self.create_publisher(Float32, '/wall_left_distance', 10)
        self.pub_wall_right = self.create_publisher(Float32, '/wall_right_distance', 10)
        self.pub_tilt_detected = self.create_publisher(Bool, '/tilt_detected', 1)
        self.pub_tilt_angle = self.create_publisher(Float32, '/tilt_angle_deg', 1)

        # 컬러·깊이 영상을 처리할 고정 해상도와 보정 실패 시 사용할 화면 중앙
        self.WIDTH = 424
        self.HEIGHT = 240
        self.CENTER_X = self.WIDTH // 2
        self.center_calibration = load_calibration(__file__)
        # 이 거리 이내의 깊이 화소만 벽 후보로 간주
        self.WALL_CLOSE_DISTANCE = 0.20
        # 산발적인 깊이 노이즈를 벽으로 오인하지 않기 위한 최소 픽셀 수
        self.WALL_MIN_PIXELS = 25
        # 벽이 없을 때 토픽으로 보내는 충분히 큰 관례값
        self.NO_WALL_DISTANCE = 9.9

        self.pipeline = rs.pipeline()
        config = rs.config()
        # 가속도·자이로 융합과 임계값 판정을 담당하는 순수 Python 필터
        self.tilt_detector = TiltDetector()

        # 같은 시야의 컬러와 깊이를 초당 30장씩 요청
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        # D435i 가속도계와 자이로 원시 스트림을 함께 요청
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

        try:
            profile = self.pipeline.start(config)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.get_logger().info('RealSense started')
        except Exception as e:
            self.get_logger().error(f'Camera fail: {e}')
            return

        if self.center_calibration is None:
            self.get_logger().warn(
                'Center calibration unavailable; using image center as fallback.'
            )
        else:
            self.get_logger().info(
                'Center calibration loaded: '
                f'{self.center_calibration["path"]} '
                f'(RMSE={self.center_calibration["rmse_px"]})'
            )
        self.align = rs.align(rs.stream.color)

        # 초록 후보를 추출하기 위한 하한·상한값
        self.lower_green = np.array([50, 100, 50])
        self.upper_green = np.array([90, 255, 255])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.prev_dist = 0.0
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            self.publish_tilt_info(frames)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
            green_mask = self.make_green_mask(color_image)

            self.publish_wall_info(depth_image, color_image, green_mask)
            self.publish_target_info(depth_frame, color_image, green_mask)

            cv2.imshow("ROS2 Green Tracker", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Frame error: {e}')

    def publish_tilt_info(self, frames):
        samples = []

        gyro_frame = frames.first_or_default(rs.stream.gyro)
        if gyro_frame:
            gyro = gyro_frame.as_motion_frame().get_motion_data()
            samples.append((gyro_frame.get_timestamp() / 1000.0, 'gyro', gyro))

        accel_frame = frames.first_or_default(rs.stream.accel)
        if accel_frame:
            accel = accel_frame.as_motion_frame().get_motion_data()
            samples.append((accel_frame.get_timestamp() / 1000.0, 'accel', accel))

        status = self.tilt_detector.status()
        for timestamp, stream_name, motion in sorted(samples, key=lambda sample: sample[0]):
            values = (motion.x, motion.y, motion.z)
            if stream_name == 'gyro':
                status = self.tilt_detector.update_gyro(values, timestamp)
            else:
                status = self.tilt_detector.update_accel(values, timestamp)

        self.pub_tilt_angle.publish(Float32(data=float(status.angle_deg)))
        self.pub_tilt_detected.publish(Bool(data=bool(status.tilted)))

    def make_green_mask(self, color_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def publish_wall_info(self, depth_image, color_image, green_mask):
        valid_depth = np.logical_and(depth_image > 0.0, depth_image < 4.0)
        close_wall = np.logical_and(valid_depth, depth_image <= self.WALL_CLOSE_DISTANCE)
        green_block = cv2.dilate(green_mask, self.kernel, iterations=2)
        close_wall = np.logical_and(close_wall, green_block == 0)

        if int(np.count_nonzero(close_wall)) < self.WALL_MIN_PIXELS:
            self.pub_wall_distance.publish(Float32(data=self.NO_WALL_DISTANCE))
            self.pub_wall_left.publish(Float32(data=self.NO_WALL_DISTANCE))
            self.pub_wall_right.publish(Float32(data=self.NO_WALL_DISTANCE))
            return

        _ys, xs = np.where(close_wall)
        left_x = int(xs.min())
        right_x = int(xs.max())
        left_distance = self.edge_distance(depth_image, close_wall, left_x)
        right_distance = self.edge_distance(depth_image, close_wall, right_x)
        nearest_distance = float(np.min(depth_image[close_wall]))

        self.pub_wall_distance.publish(Float32(data=nearest_distance))
        self.pub_wall_left.publish(Float32(data=left_distance))
        self.pub_wall_right.publish(Float32(data=right_distance))

        cv2.line(color_image, (left_x, 0), (left_x, self.HEIGHT), (0, 0, 255), 1)
        cv2.line(color_image, (right_x, 0), (right_x, self.HEIGHT), (0, 0, 255), 1)
        cv2.putText(
            color_image,
            f"W L:{left_distance:.2f} R:{right_distance:.2f}",
            (8, self.HEIGHT - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
        )

    def edge_distance(self, depth_image, close_wall, edge_x):
        x_min = max(0, edge_x - 3)
        x_max = min(self.WIDTH, edge_x + 4)
        edge_mask = close_wall[:, x_min:x_max]
        edge_depth = depth_image[:, x_min:x_max][edge_mask]
        if edge_depth.size == 0:
            return self.NO_WALL_DISTANCE
        return float(np.median(edge_depth))

    def publish_target_info(self, depth_frame, color_image, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_msg = Bool()
        found_msg.data = False

        cv2.line(
            color_image,
            (self.CENTER_X, 0),
            (self.CENTER_X, self.HEIGHT),
            (100, 100, 100),
            1,
        )
        #혹시 모르니 초록색 제한두기(픽셀 수나 만약 초록색이 2개 이산 인식되도 더 큰 쪽으로 목표로 인식)
        if contours:
            target = max(contours, key=cv2.contourArea)

            if cv2.contourArea(target) > 30:
                # 목표 외곽을 감싸는 사각형과 대표 중심 화소를 계산한다.
                x, y, w, h = cv2.boundingRect(target)
                cx = x + w // 2
                cy = y + h // 2

                # 목표 중심의 정렬된 깊이를 미터 단위로 읽는다.
                dist = depth_frame.get_distance(cx, cy)

                # 깊이가 없거나 범위를 벗어나면 직전 유효 거리로 대체한다.
                if dist == 0 or dist > 4.0:
                    dist = self.prev_dist
                else:
                    # 새 거리 70%, 이전 거리 30%를 섞어 거리 흔들림을 완화한다.
                    dist = 0.7 * dist + 0.3 * self.prev_dist
                    self.prev_dist = dist

                # 캘리브레이션이 측정한 거리별 로봇 정면 중심선을 현재 깊이에 맞춰 구한다.
                # 보정 파일을 읽지 못했거나 깊이가 0이면 helper가 화면 중앙으로 되돌린다.
                calibrated_center = calibrated_center_x(
                    self.center_calibration,
                    dist,
                    self.CENTER_X,
                    self.WIDTH,
                )
                # 보정된 로봇 정면 기준선과 목표의 수평 차이를 계산한다.
                rel_x = cx - calibrated_center
                # 화면 반폭으로 나눠 조향용 정규화 오차를 만든다.
                error_val = rel_x / self.CENTER_X

                # Brain의 조향·속도 판단에 필요한 오차와 거리를 발행한다.
                self.pub_error.publish(Float32(data=float(error_val)))
                self.pub_distance.publish(Float32(data=float(dist)))
                found_msg.data = True

                # 보정 기준선, 목표 박스, 목표 중심을 디버그 영상에 표시한다.
                cv2.line(
                    color_image,
                    (calibrated_center, 0),
                    (calibrated_center, self.HEIGHT),
                    (255, 255, 255),
                    1,
                )
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)

                txt = f"E:{error_val:.2f} D:{dist:.2f}m"
                cv2.putText(
                    color_image,
                    txt,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

        self.pub_found.publish(found_msg)

    def destroy(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = GreenTrackerDepth()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('stop')
    finally:
        if node is not None:
            node.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
