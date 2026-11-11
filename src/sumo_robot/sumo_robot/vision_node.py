import math

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from .center_calibration import calibrated_center_x, load_calibration
from .hfsm_brain.robot_settings import load_section

VISION_KEYS = (
    'WIDTH', 'HEIGHT', 'WALL_CLOSE_DISTANCE', 'WALL_MIN_PIXELS',
    'WALL_VERTICAL_SAMPLES', 'WALL_VERTICAL_REQUIRED', 'WALL_VERTICAL_SPAN',
    'WALL_VERTICAL_DEPTH_TOLERANCE', 'WALL_VERTICAL_MAX_MISSING', 'WALL_MIN_HEIGHT',
    'WALL_MAX_DISTANCE', 'FLOOR_PLANE_TOLERANCE', 'FLOOR_SAMPLE_STEP', 'NO_WALL_DISTANCE',
    'stream_fps', 'green_lower_hsv', 'green_upper_hsv', 'green_min_area',
    'target_max_distance', 'distance_new_weight', 'morphology_kernel_size',
)


class GreenTrackerDepth(Node):
    def __init__(self):
        super().__init__('green_tracker_depth')

        self.pub_error = self.create_publisher(Float32, '/target_error', 10)
        self.pub_found = self.create_publisher(Bool, '/target_found', 10)
        self.pub_distance = self.create_publisher(Float32, '/target_distance', 10)
        self.pub_wall_distance = self.create_publisher(Float32, '/wall_distance', 10)
        self.pub_wall_left = self.create_publisher(Float32, '/wall_left_distance', 10)
        self.pub_wall_right = self.create_publisher(Float32, '/wall_right_distance', 10)
        self.pub_wall_observation = self.create_publisher(Vector3Stamped, '/wall_observation', 1)

        # 모든 튜닝값은 robot_settings.yaml 의 vision 섹션에서 읽는다.
        cfg = load_section('vision', VISION_KEYS)
        self.WIDTH = int(cfg['WIDTH'])
        self.HEIGHT = int(cfg['HEIGHT'])
        # 보정 실패 시 사용할 화면 중앙
        self.CENTER_X = self.WIDTH // 2
        self.center_calibration = load_calibration(__file__)
        # 이 거리 이내의 깊이 화소만 벽 후보로 간주
        self.WALL_CLOSE_DISTANCE = float(cfg['WALL_CLOSE_DISTANCE'])
        # 산발적인 깊이 노이즈를 벽으로 오인하지 않기 위한 최소 픽셀 수
        self.WALL_MIN_PIXELS = int(cfg['WALL_MIN_PIXELS'])
        # 후보의 서로 다른 가로 위치 최대 N곳 중 R곳에서 세로 면을 확인한다.
        self.WALL_VERTICAL_SAMPLES = int(cfg['WALL_VERTICAL_SAMPLES'])
        self.WALL_VERTICAL_REQUIRED = int(cfg['WALL_VERTICAL_REQUIRED'])
        # 기준 화소에서 위로 SPAN칸 검사한다. RGB 밝기가 아니라 깊이값을 비교한다.
        self.WALL_VERTICAL_SPAN = int(cfg['WALL_VERTICAL_SPAN'])
        # 한 검사 구간의 깊이가 기준점과 이 값(m) 이내여야 같은 면으로 간주한다.
        self.WALL_VERTICAL_DEPTH_TOLERANCE = float(cfg['WALL_VERTICAL_DEPTH_TOLERANCE'])
        # 구간 중간의 깊이 누락 허용 칸 수
        self.WALL_VERTICAL_MAX_MISSING = int(cfg['WALL_VERTICAL_MAX_MISSING'])
        # Floor-relative obstacle height and plane-fit tolerance, in meters.
        self.WALL_MIN_HEIGHT = float(cfg['WALL_MIN_HEIGHT'])
        # 벽 후보 계산에 쓰는 유효 깊이 상한(m)
        self.WALL_MAX_DISTANCE = float(cfg['WALL_MAX_DISTANCE'])
        self.FLOOR_PLANE_TOLERANCE = float(cfg['FLOOR_PLANE_TOLERANCE'])
        self.FLOOR_SAMPLE_STEP = int(cfg['FLOOR_SAMPLE_STEP'])
        # 벽이 없을 때 토픽으로 보내는 충분히 큰 관례값
        self.NO_WALL_DISTANCE = float(cfg['NO_WALL_DISTANCE'])
        self.stream_fps = int(cfg['stream_fps'])
        self.green_min_area = float(cfg['green_min_area'])
        self.target_max_distance = float(cfg['target_max_distance'])
        self.distance_new_weight = float(cfg['distance_new_weight'])
        if not 0.0 <= self.distance_new_weight <= 1.0:
            raise ValueError('vision.distance_new_weight must be between 0 and 1')

        self.pipeline = rs.pipeline()
        config = rs.config()

        # 같은 시야의 컬러와 깊이를 stream_fps 로 요청
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.stream_fps)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.stream_fps)

        try:
            profile = self.pipeline.start(config)
        except Exception as e:
            self.get_logger().error(f'pipeline.start failed: {e}')
            raise

        try:
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.floor_x_factor = (np.arange(self.WIDTH) - intrinsics.ppx) / intrinsics.fx
            self.floor_y_factor = (np.arange(self.HEIGHT) - intrinsics.ppy) / intrinsics.fy
            self.floor_rng = np.random.default_rng()
        except Exception as e:
            self.get_logger().error(f'Camera metadata failed: {e}')
            self.pipeline.stop()
            raise

        self.get_logger().info('RealSense started')

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
        self.lower_green = np.array(cfg['green_lower_hsv'], dtype=np.uint8)
        self.upper_green = np.array(cfg['green_upper_hsv'], dtype=np.uint8)
        kernel_size = int(cfg['morphology_kernel_size'])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))

        self.prev_dist = 0.0
        self.wall_stamp_fallback_logged = False
        self.timer = self.create_timer(1.0 / self.stream_fps, self.process_frame)

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            received_stamp = self.get_clock().now().to_msg()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
            green_mask = self.make_green_mask(color_image)

            nearest, left, right = self.publish_wall_info(depth_image, green_mask)
            observation = Vector3Stamped()
            observation.header.stamp = self.wall_frame_stamp(depth_frame, received_stamp)
            observation.vector.x = left
            observation.vector.y = right
            observation.vector.z = nearest
            self.pub_wall_observation.publish(observation)
            self.publish_target_info(depth_frame, green_mask)

        except Exception as e:
            self.get_logger().error(f'Frame error: {e}')

    def wall_frame_stamp(self, depth_frame, received_stamp):
        stamp = received_stamp
        timestamp_ms = depth_frame.get_timestamp()
        if (depth_frame.get_frame_timestamp_domain() == rs.timestamp_domain.global_time
                and math.isfinite(timestamp_ms) and timestamp_ms > 0.0):
            frame_ns = round(timestamp_ms * 1e6)
            stamp.sec, stamp.nanosec = divmod(frame_ns, 1_000_000_000)
        elif not self.wall_stamp_fallback_logged:
            self.get_logger().warning(
                'RealSense capture time unavailable; wall observation uses frame receipt time'
            )
            self.wall_stamp_fallback_logged = True
        return stamp

    def make_green_mask(self, color_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def publish_wall_info(self, depth_image, green_mask):
        """Broadcast floor height across the image without repeated index copies."""
        green_block = cv2.dilate(green_mask, self.kernel, iterations=2)
        surface_mask = (depth_image > 0) & (depth_image < self.WALL_MAX_DISTANCE) & (green_block == 0)
        gap_mask = (~np.isfinite(depth_image) | (depth_image <= 0)) & (green_block == 0)
        plane = self.estimate_floor_plane(depth_image)
        if plane is not None:
            nx, ny, nz, offset = plane
            height = -(nx * self.floor_x_factor[None, :] * depth_image + ny * self.floor_y_factor[:, None] * depth_image + nz * depth_image + offset)
            surface_mask &= height >= self.WALL_MIN_HEIGHT
        close = self.filter_vertical_walls(depth_image, surface_mask & (depth_image <= self.WALL_CLOSE_DISTANCE), surface_mask, gap_mask)
        nearest = left = right = self.NO_WALL_DISTANCE
        if np.any(close):
            _, xs = np.where(close)
            left = self.edge_distance(depth_image, close, int(xs.min()))
            right = self.edge_distance(depth_image, close, int(xs.max()))
            nearest = float(np.min(depth_image[close]))
        self.pub_wall_distance.publish(Float32(data=nearest))
        self.pub_wall_left.publish(Float32(data=left))
        self.pub_wall_right.publish(Float32(data=right))
        return (nearest, left, right)

    def filter_vertical_walls(self, depth_image, close_wall, surface_mask, gap_mask):
        """Evaluate vertical spans only in the columns used by final wall tests."""
        accepted = np.zeros_like(close_wall, dtype=bool)
        offset = self.WALL_VERTICAL_SPAN
        span = offset + 1
        if depth_image.shape[0] < span or not np.any(close_wall):
            return accepted
        count, labels, stats, _ = cv2.connectedComponentsWithStats(close_wall.astype(np.uint8), connectivity=8)
        candidates = []
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < self.WALL_MIN_PIXELS:
                continue
            x, y, width, height, _ = stats[label]
            component = labels[y:y + height, x:x + width] == label
            columns = np.flatnonzero(np.any(component, axis=0)) + x
            if columns.size < self.WALL_VERTICAL_REQUIRED:
                continue
            indices = np.linspace(0, columns.size - 1, min(self.WALL_VERTICAL_SAMPLES, columns.size), dtype=int)
            candidates.append((label, columns[indices]))
        if not candidates:
            return accepted
        columns = np.unique(np.concatenate([cols for _, cols in candidates]))
        finite = np.where(np.isfinite(depth_image[:, columns]), depth_image[:, columns], 0.0)
        surfaces = surface_mask[:, columns]
        gaps = gap_mask[:, columns]
        anchors = finite[offset:]
        passed = close_wall[offset:, columns].copy()
        missing = np.zeros_like(anchors, dtype=np.int32)
        for shift in range(1, span):
            matching = surfaces[offset - shift:len(finite) - shift] & (np.abs(finite[offset - shift:len(finite) - shift] - anchors) <= self.WALL_VERTICAL_DEPTH_TOLERANCE)
            gap = gaps[offset - shift:len(finite) - shift]
            missing += gap
            passed &= matching | gap
            passed &= missing <= self.WALL_VERTICAL_MAX_MISSING
            if shift == offset:
                passed &= matching
            if not np.any(passed):
                return accepted
        for label, selected in candidates:
            verified = []
            for column in selected:
                index = np.searchsorted(columns, column)
                column_passed = passed[:, index] & (labels[offset:, column] == label)
                if not np.any(column_passed):
                    continue
                supported = np.convolve(column_passed.astype(np.int32), np.ones(span, dtype=np.int32), mode='full') > 0
                supported &= labels[:, column] == label
                verified.append((column, supported))
            if len(verified) >= self.WALL_VERTICAL_REQUIRED:
                for column, supported in verified:
                    accepted[:, column] |= supported
        return accepted

    def estimate_floor_plane(self, depth_image):
        """Evaluate the same 32 RANSAC samples in a NumPy batch."""
        step = self.FLOOR_SAMPLE_STEP
        ys, xs = np.mgrid[self.HEIGHT // 2:self.HEIGHT:step, 0:self.WIDTH:step]
        z = depth_image[ys, xs]
        valid = (z >= 0.1) & (z <= 2.0)
        if np.count_nonzero(valid) < 50:
            return None
        ys, xs, z = (ys[valid], xs[valid], z[valid])
        points = np.column_stack((self.floor_x_factor[xs] * z, self.floor_y_factor[ys] * z, z))
        indices = np.array([self.floor_rng.choice(len(points), 3, replace=False) for _ in range(32)])
        normals = np.cross(points[indices[:, 1]] - points[indices[:, 0]], points[indices[:, 2]] - points[indices[:, 0]])
        norms = np.linalg.norm(normals, axis=1)
        usable = norms >= 1e-08
        normals /= np.where(usable, norms, 1)[:, None]
        normals[normals[:, 1] < 0] *= -1
        offsets = -np.einsum('ij,ij->i', normals, points[indices[:, 0]])
        usable &= (normals[:, 1] >= 0.5) & (offsets > -0.6) & (offsets < -0.03)
        distances = np.abs(np.einsum('pj,nj->np', points, normals) + offsets[:, None])
        inliers = distances < self.FLOOR_PLANE_TOLERANCE
        counts = np.count_nonzero(inliers, axis=1)
        counts[~usable] = 0
        best = int(np.argmax(counts))
        if counts[best] < max(50, int(len(points) * 0.1)):
            return None
        floor_points = points[inliers[best]]
        if np.ptp(floor_points[:, 2]) < 0.05:
            return None
        center = floor_points.mean(axis=0)
        try:
            _, _, vectors = np.linalg.svd(floor_points - center, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        normal = vectors[-1]
        if normal[1] < 0:
            normal = -normal
        offset = -float(normal @ center)
        if normal[1] < 0.5 or not -0.6 < offset < -0.03:
            return None
        return (*normal, offset)

    def edge_distance(self, depth_image, close_wall, edge_x):
        x_min = max(0, edge_x - 3)
        x_max = min(self.WIDTH, edge_x + 4)
        edge_mask = close_wall[:, x_min:x_max]
        edge_depth = depth_image[:, x_min:x_max][edge_mask]
        if edge_depth.size == 0:
            return self.NO_WALL_DISTANCE
        return float(np.median(edge_depth))

    def publish_target_info(self, depth_frame, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_msg = Bool()
        found_msg.data = False

        #혹시 모르니 초록색 제한두기(픽셀 수나 만약 초록색이 2개 이산 인식되도 더 큰 쪽으로 목표로 인식)
        if contours:
            target = max(contours, key=cv2.contourArea)

            if cv2.contourArea(target) > self.green_min_area:
                # 목표 외곽을 감싸는 사각형과 대표 중심 화소를 계산한다.
                x, y, w, h = cv2.boundingRect(target)
                cx = x + w // 2
                cy = y + h // 2

                # 목표 중심의 정렬된 깊이를 미터 단위로 읽는다.
                dist = depth_frame.get_distance(cx, cy)

                # 깊이가 없거나 범위를 벗어나면 직전 유효 거리로 대체한다.
                if dist == 0 or dist > self.target_max_distance:
                    dist = self.prev_dist
                else:
                    # 새 거리와 이전 거리를 distance_new_weight 비율로 섞어 흔들림을 완화한다.
                    w = self.distance_new_weight
                    dist = w * dist + (1.0 - w) * self.prev_dist
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

        self.pub_found.publish(found_msg)

    def destroy(self):
        self.pipeline.stop()


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
