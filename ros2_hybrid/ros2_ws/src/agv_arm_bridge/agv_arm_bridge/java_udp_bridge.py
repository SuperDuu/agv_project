import json
import socket
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class JavaUdpBridge(Node):
    """UDP bridge for Java requests.

    Input packet example:
    {
      "type": "plan_pose",
      "request_id": "optional-id",
      "arm": "right",
      "target": {"x": 120.0, "y": 0.0, "z": 20.0},
      "reply_host": "127.0.0.1",
      "reply_port": 5011
    }

    This node currently publishes the request to ROS 2 and returns an
    acknowledged response. A MoveIt planner node will subscribe to
    /agv_arm/plan_requests and publish trajectories in the next step.
    """

    def __init__(self):
        super().__init__("java_udp_bridge")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 5010)
        self.declare_parameter("default_reply_host", "127.0.0.1")
        self.declare_parameter("default_reply_port", 5011)

        self.listen_host = self.get_parameter("listen_host").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.default_reply_host = self.get_parameter("default_reply_host").value
        self.default_reply_port = int(self.get_parameter("default_reply_port").value)

        self.publisher = self.create_publisher(String, "/agv_arm/plan_requests", 10)
        self.subscription = self.create_subscription(
            String, "/agv_arm/plan_responses", self.handle_planner_response, 10
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.listen_host, self.listen_port))
        self.socket.setblocking(False)
        self.timer = self.create_timer(0.02, self.poll_socket)

        self.pending_requests = {}
        self.get_logger().info(f"Listening for Java UDP requests on {self.listen_host}:{self.listen_port}")

    def poll_socket(self):
        # Clean up stale requests (timeout > 5.0 seconds)
        now = time.time()
        stale_ids = [rid for rid, (_, _, _, t) in self.pending_requests.items() if now - t > 5.0]
        for rid in stale_ids:
            _, reply_host, reply_port, _ = self.pending_requests.pop(rid)
            timeout_response = {
                "type": "plan_response",
                "request_id": rid,
                "ok": False,
                "error": "Planning timeout from ROS 2 backend",
                "stamp": now,
            }
            try:
                self.socket.sendto(json.dumps(timeout_response).encode("utf-8"), (reply_host, reply_port))
            except Exception:
                pass

        while True:
            try:
                data, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"UDP receive failed: {exc}")
                return

            try:
                request = json.loads(data.decode("utf-8"))
                response = self.handle_request(request, address)
                if response is None:
                    continue
            except Exception as exc:
                response = {
                    "type": "plan_response",
                    "ok": False,
                    "error": str(exc),
                    "stamp": time.time(),
                }

            reply_host = response.pop("_reply_host", address[0])
            reply_port = int(response.pop("_reply_port", self.default_reply_port))
            self.socket.sendto(json.dumps(response).encode("utf-8"), (reply_host, reply_port))

    def handle_request(self, request, address):
        request_id = request.get("request_id") or str(uuid.uuid4())
        request["request_id"] = request_id
        request.setdefault("received_from", {"host": address[0], "port": address[1]})
        request.setdefault("stamp", time.time())

        reply_host = request.get("reply_host", self.default_reply_host)
        reply_port = int(request.get("reply_port", self.default_reply_port))
        
        # Save pending request details to reply once the planner finishes
        self.pending_requests[request_id] = (address[0], reply_host, reply_port, time.time())

        msg = String()
        msg.data = json.dumps(request)
        self.publisher.publish(msg)
        return None

    def handle_planner_response(self, msg):
        try:
            response = json.loads(msg.data)
            request_id = response.get("request_id")
            if not request_id or request_id not in self.pending_requests:
                return

            _, reply_host, reply_port, _ = self.pending_requests.pop(request_id)
            self.socket.sendto(json.dumps(response).encode("utf-8"), (reply_host, reply_port))
            self.get_logger().info(f"Replied to request {request_id} over UDP")
        except Exception as exc:
            self.get_logger().error(f"Error handling planner response: {exc}")

    def destroy_node(self):
        try:
            self.socket.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = JavaUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
