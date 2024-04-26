#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

#include <libserial/SerialPort.h>

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("simple_serial_receiver")
    {
        declare_parameter<std::string>("port", "/dev/ttyACM0");
        port_ = get_parameter("port").as_string();
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        timer1_ = create_wall_timer(0.5s, std::bind(&SimpleSerialReceiver::timerCallback, this));
        timer2_= create_wall_timer(0.5s, std::bind(&SimpleSerialReceiver::timerCallback_write, this));
    }

    ~SimpleSerialReceiver()
    {
        arduino_.Close();
    }

    void timerCallback()
    {
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        {
            auto message = std_msgs::msg::String();
            arduino_.ReadLine(message.data);
            RCLCPP_INFO(get_logger(), "Received message: %s", message.data.c_str());
        }
    }

    void timerCallback_write()
    {
        std_msgs::msg::String message;
        message.data = "rp01.00,ln01.00,";
        RCLCPP_INFO(get_logger(), "Write message: %s", message.data.c_str() );
        arduino_.Write(message.data);
    }

private:
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  std::string port_;
  LibSerial::SerialPort arduino_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}