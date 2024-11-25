#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"

class MoverNode : public rclcpp::Node
{
public:
    MoverNode()
        : Node("mover_node")
    {
        // Subscriptions to relevant topics
        boat_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/boat_position_data", 10,
            std::bind(&MoverNode::boat_pose_callback, this, std::placeholders::_1));
        turbines_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/turbines_data", 10,
            std::bind(&MoverNode::turbines_callback, this, std::placeholders::_1));
        yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/boat_yaw", 10,
            std::bind(&MoverNode::yaw_callback, this, std::placeholders::_1));
    }

private:
    // Callback for boat pose data
    void boat_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Boat Position -> X: %.2f, Y: %.2f, Z: %.2f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        // update local boat position
        boat_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    }

    // Callback for turbines data
    void turbines_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received %zu turbines positions:", msg->poses.size());
        for (const auto &pose : msg->poses)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Turbine -> X: %.2f, Y: %.2f, Z: %.2f",
                        pose.position.x, pose.position.y, pose.position.z);
        }
    }

    // Callback for yaw data
    void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Boat Yaw -> %.2f radians", msg->data);
    }

    // ROS 2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr boat_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr turbines_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;

    // boat position
    struct Position
    {
        double x, y, z;
    };
    Position boat_position_;

    // turbines position
    std::vector<geometry_msgs::msg::PoseArray> turbines_cache;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoverNode>());
    rclcpp::shutdown();
    return 0;
}
