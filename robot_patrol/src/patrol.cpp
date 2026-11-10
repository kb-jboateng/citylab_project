#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "algorithm"
#include "cmath"
#include "limits"

class Patrol : public rclcpp::Node
{
    public:
        Patrol(): Node("patrol_node")
        {
            auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

            // Create laser subscriber
            laser_subsriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/laser_scan",
                qos,
                std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subsriber_;


        void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            const double front_half_angle = 20.0 * M_PI / 180.0;
            const float obstacle_dist = 0.35f;

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