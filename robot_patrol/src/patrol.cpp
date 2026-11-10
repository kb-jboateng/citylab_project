#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
            float min_left  = std::numeric_limits<float>::infinity();
            float min_right = std::numeric_limits<float>::infinity();

            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                angle = std::atan2(std::sin(angle), std::cos(angle));

                if (angle < -M_PI_2 || angle > M_PI_2)
                    continue;

                float r = msg->ranges[i];
                if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
                    continue;

                if (angle >= 0.0)
                    min_left = std::min(min_left, r);
                else
                    min_right = std::min(min_right, r);
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