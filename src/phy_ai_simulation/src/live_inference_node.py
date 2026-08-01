#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
import torch
import numpy as np
from phy_ai_simulation.pinn_model import PhysicsInformedDronePilot

class LivePinnInferenceNode(Node):
    def __init__(self):
        super().__init__('live_pinn_inference_node')
        self.pinn_pilot = PhysicsInformedDronePilot(input_dim=6, output_dim=4)
        self.pinn_pilot.eval()
        torch.set_grad_enabled(False)
        
        self.telemetry_sub = self.create_subscription(
            TwistStamped, '/pinn/input_state', self.telemetry_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/pinn_drone/cmd_vel', 10)
        self.get_logger().info('Live PyTorch PINN Inference Node running.')

    def telemetry_callback(self, msg: TwistStamped):
        state_array = np.array([
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
        ], dtype=np.float32)
        state_tensor = torch.from_numpy(state_array).unsqueeze(0)
        predicted_thrusts = self.pinn_pilot(state_tensor).numpy()[0]
        
        cmd_msg = Twist()
        cmd_msg.linear.z = float(np.sum(predicted_thrusts) - (1.5 * 9.81))
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivePinnInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
