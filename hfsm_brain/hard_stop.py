from rclpy.node import Node
from std_srvs.srv import Trigger


class Hard_Stop_Gateway:

    def __init__(self, node: Node):
        self.node = node
        self.hard_stop_client = node.create_client(Trigger, "/motor_node/hard_stop")
        self.release_client = node.create_client(
            Trigger, "/motor_node/release_hard_stop"
        )
        self.expected = False
        self.hard_stop_future = None
        self.release_future = None
        self.last_release_attempt = -1.0
        self.last_service_warning = -1.0

    def _warn(self, now: float, message: str) -> None:
        if now - self.last_service_warning >= 1.0:
            self.last_service_warning = now
            self.node.get_logger().error(message)

    def request(self, now: float) -> None:
        if self.expected or self.hard_stop_future is not None:
            return
        if not self.hard_stop_client.service_is_ready():
            self._warn(now, "Hard stop service is unavailable: /motor_node/hard_stop")
            return

        self.expected = True
        self.hard_stop_future = self.hard_stop_client.call_async(Trigger.Request())
        self.hard_stop_future.add_done_callback(self._hard_stop_response)

    def _hard_stop_response(self, future) -> None:
        self.hard_stop_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.expected = False
            self.node.get_logger().error(f"Hard stop service failed: {exc}")
            return
        if not response.success:
            self.expected = False
            self.node.get_logger().error(f"Hard stop rejected: {response.message}")
            return
        self.node.get_logger().warning(f"Hard stop requested: {response.message}")

    def ensure_released(self, now: float) -> bool:
        if not self.expected:
            return True
        if self.hard_stop_future is not None or self.release_future is not None:
            return False
        if now - self.last_release_attempt < 0.10:
            return False
        if not self.release_client.service_is_ready():
            self._warn(
                now,
                "Hard stop release service is unavailable: "
                "/motor_node/release_hard_stop",
            )
            return False

        self.last_release_attempt = now
        self.release_future = self.release_client.call_async(Trigger.Request())
        self.release_future.add_done_callback(self._release_response)
        return False

    def _release_response(self, future) -> None:
        self.release_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.node.get_logger().error(f"Hard stop release service failed: {exc}")
            return
        if response.success:
            self.expected = False
            self.node.get_logger().info(f"Hard stop released: {response.message}")
