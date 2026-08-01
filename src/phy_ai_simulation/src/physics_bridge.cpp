#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

class PhysicsBridge : public rclcpp::Node {
public:
    PhysicsBridge() : Node("phy_ai_physics_bridge") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/world/dve_wind_arena/model/x500_0/link/base_link/sensor/imu_sensor/imu", 10,
            std::bind(&PhysicsBridge::imu_callback, this, std::placeholders::_1));

        pinn_state_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/pinn/input_state", 10);
        RCLCPP_INFO(this->get_logger(), "Physics Bridge live telemetry ready.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto state_msg = geometry_msgs::msg::TwistStamped();
        state_msg.header = msg->header;
        state_msg.twist.linear.x = msg->linear_acceleration.x;
        state_msg.twist.linear.y = msg->linear_acceleration.y;
        state_msg.twist.linear.z = msg->linear_acceleration.z;
        state_msg.twist.angular.x = msg->angular_velocity.x;
        state_msg.twist.angular.y = msg->angular_velocity.y;
        state_msg.twist.angular.z = msg->angular_velocity.z;
        pinn_state_pub_->publish(state_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pinn_state_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhysicsBridge>());
    rclcpp::shutdown();
    return 0;
}
