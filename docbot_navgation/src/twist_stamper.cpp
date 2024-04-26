#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class TwistStamper : public rclcpp::Node
{
public:
    TwistStamper() : Node("twist_stamper")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("docbot_controller/cmd_vel", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TwistStamper::callback, this, _1));

    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
        
        twist_stamped_msg.twist = *msg;
        twist_stamped_msg.header.stamp = this->get_clock()->now();
        twist_stamped_msg.header.frame_id = "base_footprint";
        publisher_->publish(twist_stamped_msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistStamper>());
    rclcpp::shutdown();
    return 0;
}