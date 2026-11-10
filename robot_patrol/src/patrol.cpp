#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

class Patrol : public rclcpp::Node
{
    public:
        Patrol(): Node("patrol_node")
        {
            auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

            // Create laser subscriber
            laser_subsriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/fastbot_1/scan",
                qos,
                std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
            );

            // Create movement publisher
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subsriber_;


        void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            const double front_half_angle = 20.0 * M_PI / 180.0;
            const float obstacle_threshold = 0.35f;

            float min_front = std::numeric_limits<float>::infinity();
            float max_range = 0.0f;
            double max_angle = 0.0;

            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                angle = std::atan2(std::sin(angle), std::cos(angle));

                if (angle < -M_PI_2 || angle > M_PI_2)
                    continue;

                float r = msg->ranges[i];
                if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
                    continue;

                if (std::abs(angle) <= front_half_angle)
                    min_front = std::min(min_front, r);

                if (r > max_range)
                {
                    max_range = r;
                    max_angle = angle;
                }
            }

            if (min_front >= obstacle_threshold) 
                this->move_robot("forward");
            else
                this->move_robot(max_angle > 0.0 ? "left" : "right");
        }

        void move_robot(const std::string move)
        {
            auto action = geometry_msgs::msg::Twist();

            if (move == "forward")
            {
                action.linear.x = 0.1;
                action.angular.z = 0.0;
            }
            else if (move == "right" || move == "left")
            {
                action.linear.x = 0.05;
                action.angular.z = move == "right" ? -0.5 : 0.5;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Robot move action not supported");
            }

            publisher_->publish(action);
        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}