#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "std_msgs/msg/float64.hpp"
#include <chrono>


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

        // THRUSTERS PUBLISHERS
        aquabot_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
        aquabot_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);

        // Timer for rapid delta angle calculation AND THRUSTER MOVES
        delta_angle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(8), std::bind(&BoatPositionNode::calculate_delta_angles, this));

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

        //double dt = (current_time - last_gps_time_).seconds();
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

    void calculate_delta_angles()
    {
        if (cached_windturbines_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No wind turbine positions available for delta angle calculation.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Calculating delta angles at high frequency.");

        // "global" delta_angle to enable using it for thrusters
        double delta_angle;
        // also euclidian distance to enable stopping in front of windturbines
        double euclidian_distance;

        for (const auto &turbine : cached_windturbines_)
        {
            double delta_x = turbine.x - boat_position_.x;
            double delta_y = turbine.y - boat_position_.y;

            // Calculate the absolute angle to the wind turbine
            double angle_to_turbine = std::atan2(delta_y, delta_x);

            // Calculate the relative delta angle
            delta_angle = angle_to_turbine - yaw_;

            // Normalize the delta angle to [-pi, pi]
            delta_angle = std::atan2(std::sin(delta_angle), std::cos(delta_angle));

            // CALCULATE THE EUCLIDIAN DISTANCE TO THE WINDTURBINES AS WELL
            euclidian_distance = sqrt(pow(delta_x,2) + pow(delta_y,2));

            RCLCPP_INFO(this->get_logger(),
                        "Wind Turbine -> X: %.2f, Y: %.2f | Angle to Boat: %.2f radians | Delta Angle: %.2f radians | euclidian distance: %.2f",
                        turbine.x, turbine.y, angle_to_turbine, delta_angle, euclidian_distance);
        }

        // MOVE THRUSTERS
        // values
        auto left_msg = std_msgs::msg::Float64();
        auto right_msg = std_msgs::msg::Float64();

        if (euclidian_distance < 15.0)
        {
            left_msg.data = 0.0;
            right_msg.data = 0.0;
            aquabot_left_thrust_->publish(left_msg);
            aquabot_right_thrust_->publish(right_msg);
            RCLCPP_INFO(this->get_logger(), "STOP: Close to the wind turbine.");
            return;
        }
        
        if (delta_angle < 0){
            left_msg.data = 200.0;
            right_msg.data = -200.0;
            aquabot_left_thrust_->publish(left_msg);
            aquabot_right_thrust_->publish(right_msg);
            RCLCPP_INFO(this->get_logger(), "MOVE LEFT");
        }
        if (delta_angle > 0){
            left_msg.data = -200.0;
            right_msg.data = 200.0;
            aquabot_left_thrust_->publish(left_msg);
            aquabot_right_thrust_->publish(right_msg);
            RCLCPP_INFO(this->get_logger(), "MOVE RIGHT");
        }
        else {
            left_msg.data = 2000.0;
            right_msg.data = 2000.0;
            aquabot_left_thrust_->publish(left_msg);
            aquabot_right_thrust_->publish(right_msg);
            RCLCPP_INFO(this->get_logger(), "MOVE STRAIGHT");
        }
    }

    double degrees_to_radians(double degrees) { return degrees * M_PI / 180.0; }

    struct Position
    {
        double x, y, z;
    };

    // Cached wind turbine positions
    std::vector<Position> cached_windturbines_;

    // Timer for frequent delta angle calculation
    rclcpp::TimerBase::SharedPtr delta_angle_timer_;

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

    // thruster publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr aquabot_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr aquabot_right_thrust_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoatPositionNode>());
    rclcpp::shutdown();
    return 0;
}