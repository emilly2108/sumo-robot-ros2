import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class GreenTrackerDepth(Node):
    def __init__(self):
        super().__init__('green_tracker_depth')

        self.pub_error = self.create_publisher(Float32, '/target_error', 10)
        self.pub_found = self.create_publisher(Bool, '/target_found', 10)
        self.pub_distance = self.create_publisher(Float32, '/target_distance', 10)
        self.pub_wall_distance = self.create_publisher(Float32, '/wall_distance', 10)
        self.pub_wall_left = self.create_publisher(Float32, '/wall_left_distance', 10)
        self.pub_wall_right = self.create_publisher(Float32, '/wall_right_distance', 10)

        self.WIDTH = 424
        self.HEIGHT = 240
        self.CENTER_X = self.WIDTH // 2
        self.OFFSET_MM = 30
        self.WALL_CLOSE_DISTANCE = 0.20
        self.WALL_MIN_PIXELS = 25
        self.NO_WALL_DISTANCE = 9.9

        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)

        try:
            profile = self.pipeline.start(config)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.get_logger().info('RealSense started')
        except Exception as e:
            self.get_logger().error(f'Camera fail: {e}')
            return

        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx = intrinsics.fx
        self.align = rs.align(rs.stream.color)

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

            self.publish_wall_info(depth_image, color_image, green_mask)
            self.publish_target_info(depth_frame, color_image, green_mask)

            cv2.imshow("ROS2 Green Tracker", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Frame error: {e}')

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

        if contours:
            target = max(contours, key=cv2.contourArea)

            if cv2.contourArea(target) > 30:
                x, y, w, h = cv2.boundingRect(target)
                cx = x + w // 2
                cy = y + h // 2

                dist = depth_frame.get_distance(cx, cy)

                if dist == 0 or dist > 4.0:
                    dist = self.prev_dist
                else:
                    dist = 0.7 * dist + 0.3 * self.prev_dist
                    self.prev_dist = dist

                if dist > 0:
                    pixel_offset = int((self.OFFSET_MM * self.fx) / (dist * 1000))
                else:
                    pixel_offset = 0

                shifted_center_x = self.CENTER_X + pixel_offset
                rel_x = cx - shifted_center_x
                error_val = rel_x / self.CENTER_X

                self.pub_error.publish(Float32(data=float(error_val)))
                self.pub_distance.publish(Float32(data=float(dist)))
                found_msg.data = True

                cv2.line(
                    color_image,
                    (shifted_center_x, 0),
                    (shifted_center_x, self.HEIGHT),
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
