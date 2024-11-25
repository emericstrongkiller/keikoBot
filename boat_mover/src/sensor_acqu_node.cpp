#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

class BoatPositionNode : public rclcpp::Node
{
public:
    BoatPositionNode()
        : Node("boat_position_node"), ref_lat_(48.04630), ref_lon_(-4.97632), ref_alt_(0.0),
          earth_radius_(6378137.0), velocity_x_(0.0), velocity_y_(0.0), velocity_z_(0.0), angular_velocity_z_(0.0)
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/aquabot/sensors/gps/gps/fix", 10,
            std::bind(&BoatPositionNode::gps_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/aquabot/sensors/imu/imu/data", 10,
            std::bind(&BoatPositionNode::imu_callback, this, std::placeholders::_1));
        m_windturbines_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aquabot/ais_sensor/windturbines_positions", 10,
            std::bind(&BoatPositionNode::windturbines_callback, this, std::placeholders::_1));

        // Publishers
        boat_data_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/boat_position_data", 10);
        turbines_data_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/turbines_data", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/boat_yaw", 10); // Publisher for yaw

        // Timer for publishing data
        data_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&BoatPositionNode::publish_data, this));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto current_time = this->get_clock()->now();
        if (!last_imu_time_.nanoseconds())
        {
            last_imu_time_ = current_time;
            return;
        }

        // Extract quaternion and convert to roll, pitch, yaw
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);

        last_imu_time_ = current_time;
        orientation_ready_ = true;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        auto current_time = this->get_clock()->now();
        if (!last_gps_time_.nanoseconds())
        {
            last_gps_time_ = current_time;
            boat_position_ = {0.0, 0.0, 0.0};
            return;
        }

        last_gps_time_ = current_time;

        // Convert lat/lon/alt to Cartesian coordinates
        double lat_rad = degrees_to_radians(msg->latitude);
        double lon_rad = degrees_to_radians(msg->longitude);
        double ref_lat_rad = degrees_to_radians(ref_lat_);
        double ref_lon_rad = degrees_to_radians(ref_lon_);
        double new_x = earth_radius_ * std::cos(ref_lat_rad) * (lon_rad - ref_lon_rad);
        double new_y = earth_radius_ * (lat_rad - ref_lat_rad);
        double new_z = msg->altitude - ref_alt_;

        boat_position_ = {new_x, new_y, new_z};
    }

    void windturbines_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        cached_windturbines_.clear();
        for (const auto &pose : msg->poses)
        {
            double latitude = pose.position.x;
            double longitude = pose.position.y;
            double altitude = pose.position.z;

            // Convert to Cartesian coordinates and store in the cache
            double lat_rad = degrees_to_radians(latitude);
            double lon_rad = degrees_to_radians(longitude);
            double ref_lat_rad = degrees_to_radians(ref_lat_);
            double ref_lon_rad = degrees_to_radians(ref_lon_);
            double x = earth_radius_ * std::cos(ref_lat_rad) * (lon_rad - ref_lon_rad);
            double y = earth_radius_ * (lat_rad - ref_lat_rad);
            double z = altitude - ref_alt_;

            cached_windturbines_.emplace_back(Position{x, y, z});
        }
    }

    void publish_data()
    {
        // Publish boat position
        auto boat_msg = geometry_msgs::msg::PoseStamped();
        boat_msg.header.stamp = this->get_clock()->now();
        boat_msg.header.frame_id = "boat";

        boat_msg.pose.position.x = boat_position_.x;
        boat_msg.pose.position.y = boat_position_.y;
        boat_msg.pose.position.z = boat_position_.z;

        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        boat_msg.pose.orientation.x = q.x();
        boat_msg.pose.orientation.y = q.y();
        boat_msg.pose.orientation.z = q.z();
        boat_msg.pose.orientation.w = q.w();

        boat_data_pub_->publish(boat_msg);

        // Publish turbines data
        auto turbines_msg = geometry_msgs::msg::PoseArray();
        turbines_msg.header.stamp = this->get_clock()->now();
        turbines_msg.header.frame_id = "map";

        for (const auto &turbine : cached_windturbines_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = turbine.x;
            pose.position.y = turbine.y;
            pose.position.z = turbine.z;
            turbines_msg.poses.push_back(pose);
        }

        turbines_data_pub_->publish(turbines_msg);

        // Publish yaw as a separate message
        auto yaw_msg = std_msgs::msg::Float64();
        yaw_msg.data = yaw_;
        yaw_pub_->publish(yaw_msg);
    }

    double degrees_to_radians(double degrees) { return degrees * M_PI / 180.0; }

    struct Position
    {
        double x, y, z;
    };

    // Cached wind turbine positions
    std::vector<Position> cached_windturbines_;

    // Timer for publishing data
    rclcpp::TimerBase::SharedPtr data_pub_timer_;

    // Reference spherical coordinates
    double ref_lat_, ref_lon_, ref_alt_, earth_radius_;
    Position boat_position_;
    double roll_, pitch_, yaw_;

    // Flags and time tracking
    bool orientation_ready_ = false;
    rclcpp::Time last_gps_time_, last_imu_time_;

    // Velocities
    double velocity_x_, velocity_y_, velocity_z_;
    double angular_velocity_z_;

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_windturbines_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Data publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr boat_data_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr turbines_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_; // New publisher for yaw
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoatPositionNode>());
    rclcpp::shutdown();
    return 0;
}
