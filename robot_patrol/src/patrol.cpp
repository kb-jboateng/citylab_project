#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription_options.hpp"
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

            // Create callback group
            callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = callback_group_;

            // Create laser subscriber
            laser_subsriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/fastbot_1/scan",
                qos,
                std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1),
                sub_options
            );

            // Create movement publisher
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

            // Create control callback loop at 10Hz
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&Patrol::control_callback, this),
                callback_group_);
        }

    private:
        double max_angle_ = 0.0;
        const float obstacle_threshold_ = 0.35f;
        float min_front_ = std::numeric_limits<float>::infinity();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subsriber_;


        void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            const double front_half_angle = 20.0 * M_PI / 180.0;

            float max_range = 0.0f;
            max_angle_ = 0.0;
            min_front_ = std::numeric_limits<float>::infinity();

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
                    min_front_ = std::min(min_front_, r);

                if (r > max_range)
                {
                    max_range = r;
                    max_angle_ = angle;

                    RCLCPP_INFO(this->get_logger(), "Max angle from laser callbackk %2.f", max_angle_);
                }
                RCLCPP_INFO(this->get_logger(), "Min front from laser callbackk %2.f", min_front_);
            }

        }

        void control_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Max angle from control callbackk %2.f", max_angle_);
            RCLCPP_INFO(this->get_logger(), "Min front from control callbackk %2.f", min_front_);
            if (min_front_ >= obstacle_threshold_) 
                this->move_robot("forward");
            else
                this->move_robot(max_angle_ > 0.0 ? "left" : "right");
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

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2
    );

    executor.add_node(node);

    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}