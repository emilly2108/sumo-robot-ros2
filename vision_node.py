import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from .center_calibration import calibrated_center_x, load_calibration


class GreenTrackerDepth(Node):
    def __init__(self):
        super().__init__('green_tracker_depth')

        self.pub_error = self.create_publisher(Float32, '/target_error', 10)
        self.pub_found = self.create_publisher(Bool, '/target_found', 10)
        self.pub_distance = self.create_publisher(Float32, '/target_distance', 10)
        self.pub_wall_distance = self.create_publisher(Float32, '/wall_distance', 10)
        self.pub_wall_left = self.create_publisher(Float32, '/wall_left_distance', 10)
        self.pub_wall_right = self.create_publisher(Float32, '/wall_right_distance', 10)

        # 컬러·깊이 영상을 처리할 고정 해상도와 보정 실패 시 사용할 화면 중앙
        self.WIDTH = 424
        self.HEIGHT = 240
        self.CENTER_X = self.WIDTH // 2
        self.center_calibration = load_calibration(__file__)
        # 이 거리 이내의 깊이 화소만 벽 후보로 간주
        self.WALL_CLOSE_DISTANCE = 0.30
        # 산발적인 깊이 노이즈를 벽으로 오인하지 않기 위한 최소 픽셀 수
        self.WALL_MIN_PIXELS = 25
        # 후보의 서로 다른 가로 위치 최대 10곳 중 6곳에서 세로 면을 확인한다.
        self.WALL_VERTICAL_SAMPLES = 10
        self.WALL_VERTICAL_REQUIRED = 6
        # 기준 화소에서 위로 20칸 검사한다. RGB 밝기가 아니라 깊이값을 비교한다.
        self.WALL_VERTICAL_SPAN = 20
        # 한 검사 구간의 깊이는 기준점과 3cm 이내여야 같은 면으로 간주한다.
        self.WALL_VERTICAL_DEPTH_TOLERANCE = 0.03
        # 구간 양끝은 유효해야 하며, 중간의 깊이 누락은 최대 2칸만 허용한다.
        self.WALL_VERTICAL_MAX_MISSING = 2
        # Floor-relative obstacle height and plane-fit tolerance, in meters.
        self.WALL_MIN_HEIGHT = 0.05
        self.FLOOR_PLANE_TOLERANCE = 0.015
        self.FLOOR_SAMPLE_STEP = 6
        # 벽이 없을 때 토픽으로 보내는 충분히 큰 관례값
        self.NO_WALL_DISTANCE = 9.9

        self.pipeline = rs.pipeline()
        config = rs.config()

        # 같은 시야의 컬러와 깊이를 초당 30장씩 요청
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)

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
        self.lower_green = np.array([50, 100, 50])
        self.upper_green = np.array([90, 255, 255])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.prev_dist = 0.0
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
            green_mask = self.make_green_mask(color_image)

            self.publish_wall_info(depth_image, green_mask)
            self.publish_target_info(depth_frame, green_mask)

        except Exception as e:
            self.get_logger().error(f'Frame error: {e}')

    def make_green_mask(self, color_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def publish_wall_info(self, depth_image, green_mask):
        valid_depth = np.logical_and(depth_image > 0.0, depth_image < 4.0)
        green_block = cv2.dilate(green_mask, self.kernel, iterations=2)
        surface_mask = np.logical_and(valid_depth, green_block == 0)
        # 깊이 0/비정상 값만 짧은 누락으로 허용한다. 먼 배경이나 초록색은 누락이 아니다.
        missing_depth = np.logical_or(~np.isfinite(depth_image), depth_image <= 0.0)
        gap_mask = np.logical_and(missing_depth, green_block == 0)

        floor_plane = self.estimate_floor_plane(depth_image)
        if floor_plane is not None:
            ys, xs = np.where(surface_mask)
            if xs.size:
                z = depth_image[ys, xs]
                nx, ny, nz, offset = floor_plane
                height_above_floor = -(
                    nx * self.floor_x_factor[xs] * z
                    + ny * self.floor_y_factor[ys] * z
                    + nz * z
                    + offset
                )
                surface_mask[ys, xs] = height_above_floor >= self.WALL_MIN_HEIGHT

        close_wall = np.logical_and(surface_mask, depth_image <= self.WALL_CLOSE_DISTANCE)
        # 바닥 추정에 실패해도 면적 25개만으로 통과시키지 않고 세로 검사를 수행한다.
        # 검사 중 위쪽 점은 40cm를 약간 넘을 수 있지만 최종 거리에는 가까운 점만 쓴다.
        close_wall = self.filter_vertical_walls(
            depth_image, close_wall, surface_mask, gap_mask
        )

        if not np.any(close_wall):
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

    def filter_vertical_walls(self, depth_image, close_wall, surface_mask, gap_mask):
        # 각 후보에서 깊이가 위쪽으로 이어지는 세로줄을 검사한다. 프레임 대기는 없다.
        accepted = np.zeros_like(close_wall, dtype=bool)
        span = self.WALL_VERTICAL_SPAN + 1  # 기준 화소와 그 위 20칸.
        if depth_image.shape[0] < span or not np.any(close_wall):
            return accepted

        # 모든 후보의 세로 검사를 한 번에 계산해 작은 조각이 많아도 중복하지 않는다.
        finite_depth = np.where(np.isfinite(depth_image), depth_image, 0.0)
        offset = self.WALL_VERTICAL_SPAN
        ys, xs = np.where(close_wall)
        row_start = max(offset, int(ys.min()))
        row_stop = int(ys.max()) + 1
        if row_start >= row_stop:
            return accepted
        columns_slice = slice(int(xs.min()), int(xs.max()) + 1)
        anchors = finite_depth[row_start:row_stop, columns_slice]
        passed = close_wall[row_start:row_stop, columns_slice].copy()
        missing = np.zeros_like(anchors, dtype=np.int32)
        for shift in range(1, span):
            rows = slice(row_start - shift, row_stop - shift)
            matching = surface_mask[rows, columns_slice] & (
                np.abs(finite_depth[rows, columns_slice] - anchors)
                <= self.WALL_VERTICAL_DEPTH_TOLERANCE
            )
            gaps = gap_mask[rows, columns_slice]
            missing += gaps
            passed &= matching | gaps
            passed &= missing <= self.WALL_VERTICAL_MAX_MISSING
            if shift == offset:
                passed &= matching  # 위쪽 끝은 누락이 아닌 실제 측정점이어야 한다.
            if not np.any(passed):
                return accepted

        vertical_anchors = np.zeros_like(close_wall, dtype=bool)
        vertical_anchors[row_start:row_stop, columns_slice] = passed
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            close_wall.astype(np.uint8), connectivity=8
        )
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < self.WALL_MIN_PIXELS:
                continue
            x, y, width, height, _ = stats[label]
            component = labels[y:y + height, x:x + width] == label
            columns = np.flatnonzero(np.any(component, axis=0)) + x
            if columns.size < self.WALL_VERTICAL_REQUIRED:
                continue
            # 임의 추출 대신 좌우 끝을 포함해 고르게 뽑아 프레임별 흔들림을 줄인다.
            indices = np.linspace(
                0, columns.size - 1, min(self.WALL_VERTICAL_SAMPLES, columns.size), dtype=int
            )
            verified_columns = []
            for column in columns[indices]:
                column_passed = (
                    vertical_anchors[offset:, column]
                    & (labels[offset:, column] == label)
                )
                if not np.any(column_passed):
                    continue
                # 통과한 구간만 남긴다. 같은 덩어리에 붙은 낮은 턱까지 다시 살리지 않는다.
                supported = np.convolve(
                    column_passed.astype(np.int32), np.ones(span, dtype=np.int32), mode='full'
                ) > 0
                supported &= labels[:, column] == label
                verified_columns.append((column, supported))

            if len(verified_columns) >= self.WALL_VERTICAL_REQUIRED:
                for column, supported in verified_columns:
                    accepted[:, column] |= supported
        return accepted

    def estimate_floor_plane(self, depth_image):
        # Fit a floor-facing plane from sparse lower-image depth samples.
        step = self.FLOOR_SAMPLE_STEP
        ys, xs = np.mgrid[self.HEIGHT // 2:self.HEIGHT:step, 0:self.WIDTH:step]
        z = depth_image[ys, xs]
        valid = np.logical_and(z >= 0.10, z <= 2.0)
        if np.count_nonzero(valid) < 50:
            return None

        ys, xs, z = ys[valid], xs[valid], z[valid]
        points = np.column_stack((
            self.floor_x_factor[xs] * z,
            self.floor_y_factor[ys] * z,
            z,
        ))
        best_inliers = None
        best_count = 0
        for _ in range(32):
            indices = self.floor_rng.choice(len(points), 3, replace=False)
            normal = np.cross(
                points[indices[1]] - points[indices[0]],
                points[indices[2]] - points[indices[0]],
            )
            magnitude = np.linalg.norm(normal)
            if magnitude < 1e-8:
                continue
            normal /= magnitude
            if normal[1] < 0.0:
                normal = -normal
            offset = -float(normal @ points[indices[0]])
            if normal[1] < 0.5 or not -0.6 < offset < -0.03:
                continue
            inliers = np.abs(points @ normal + offset) < self.FLOOR_PLANE_TOLERANCE
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_inliers, best_count = inliers, count

        if best_count < max(50, int(len(points) * 0.10)):
            return None
        floor_points = points[best_inliers]
        if np.ptp(floor_points[:, 2]) < 0.05:
            return None
        center = floor_points.mean(axis=0)
        try:
            _, _, vectors = np.linalg.svd(floor_points - center, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        normal = vectors[-1]
        if normal[1] < 0.0:
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
