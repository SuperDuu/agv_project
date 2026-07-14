#!/usr/bin/env python3
"""Test the full plan_request → plan_response topic pipeline."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time

rclpy.init()
node = rclpy.create_node('test_pipeline')
pub = node.create_publisher(String, '/agv_arm/plan_requests', 10)
responses = []

def cb(msg):
    responses.append(json.loads(msg.data))

sub = node.create_subscription(String, '/agv_arm/plan_responses', cb, 10)
time.sleep(1.0)  # wait for discovery

msg = String()
msg.data = json.dumps({
    'type': 'plan_pose',
    'request_id': 't99',
    'arm': 'right',
    'target': {'x': 200.0, 'y': 0.0, 'z': 300.0},
    'current_joints': [0.0, 0.0, 20.0, -35.0, 0.0, 0.0],
    'preferred_config': '+',
    'reply_host': '127.0.0.1',
    'reply_port': 9999
})
pub.publish(msg)
print('Published plan_pose request')

deadline = time.time() + 8.0
while time.time() < deadline and not responses:
    rclpy.spin_once(node, timeout_sec=0.1)

if responses:
    r = responses[0]
    traj = r.get('trajectory', [])
    print(f"ok={r.get('ok')} frames={len(traj)} error={r.get('error','')}")
else:
    print('NO RESPONSE in 8s')

node.destroy_node()
rclpy.shutdown()
