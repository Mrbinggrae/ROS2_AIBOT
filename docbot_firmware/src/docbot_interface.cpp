#include "docbot_firmware/docbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <iostream> 
#include <std_msgs/msg/string.hpp>


namespace docbot_firmware
{
DocbotInterface::DocbotInterface()
{

}

DocbotInterface::~DocbotInterface()
{
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(
                rclcpp::get_logger("DocbotInterface"), 
                "포트 연결을 종료하는 중에 문제가 발생했습니다. " << port_);
        }
        
    }
}

CallbackReturn DocbotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_= info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("DocbotInterface"), "직렬 포트가 제공되지 않습니다! 중단하는 중");
        return CallbackReturn::FAILURE;
    }

    try
    {
        wheel_radius_= std::stod(info_.hardware_parameters.at("wheel_radius"));
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("DocbotInterface"), "바퀴 반지름을 알 수 없습니다.");
        return CallbackReturn::FAILURE;
    }

    // 백터의 용량을 미리 확보한다.
    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DocbotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION, &position_states_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DocbotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
     for(size_t i = 0; i < info_.joints.size(); i++)
     {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));


     }

     return command_interfaces;
}

CallbackReturn DocbotInterface::on_activate(const rclcpp_lifecycle::State &) 
{
    RCLCPP_INFO(rclcpp::get_logger("DocbotInterface"), "로봇 하드웨어 시작 중...");
    velocity_commands_ = {0.0, 0.0};
    position_states_ = {0.0, 0.0};
    velocity_states_ = {0.0, 0.0};


    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(
            rclcpp::get_logger("DocbotInterface"),
            "포트와 상호작용하는 중에 문제가 발생했습니다. " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("DocbotInterface"), "하드웨어가 명령을 받을 준비가 시작되었습니다.");
    return CallbackReturn::SUCCESS;
}
CallbackReturn DocbotInterface::on_deactivate(const rclcpp_lifecycle::State &) 
{
    RCLCPP_INFO(rclcpp::get_logger("DocbotInterface"), "로봇 하드웨어를 중지하는 중...");
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(
                rclcpp::get_logger("DocbotInterface"),
                "포트를 닫는 중에 문제가 발생했습니다." << port_);
        }
    }
    return CallbackReturn::FAILURE;

}

hardware_interface::return_type DocbotInterface::read(const rclcpp::Time & , const rclcpp::Duration &) 
{
    if (arduino_.IsDataAvailable())
    {
        std::string message;
        arduino_.ReadLine(message);
        std::stringstream ss(message);
        std::string res;

        //RCLCPP_INFO_STREAM(rclcpp::get_logger("DocbotInterface"), "read Serial - " << ss.str());
        int multiplier = 1;
        while(std::getline(ss, res, ','))
        {
            multiplier = res.at(1) == 'p' ? 1 : -1;
            if(res.at(0) == 'r') // 오른쪽 바퀴인지 확인
            {
                // substr는 문자열에서 첫번째 매개변수에 있는 문자부터 두번째 매개변수의 문자까지 추출한다.
                velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(0) +=  velocity_states_.at(0) * wheel_radius_ * 5.5;

                // RCLCPP_INFO(rclcpp::get_logger("vel"), "r : %lf",right_wheel_distance);
                // RCLCPP_INFO(rclcpp::get_logger("p"), "r position : %lf", position_states_.at(0));
            }

            else if(res.at(0) == 'l')
            {
                // 0은 right 1은 left
                velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(1) +=  velocity_states_.at(1) * wheel_radius_ * 5.5;
                //RCLCPP_INFO(rclcpp::get_logger("vel"), "l : %lf",left_wheel_distance);
            }
        }

        //RCLCPP_INFO(rclcpp::get_logger("encoder"), "er : %lf, el : %lf",velocity_states_.at(0), velocity_states_.at(1));
       
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DocbotInterface::write(const rclcpp::Time & , const rclcpp::Duration & ) 
{
    std::stringstream message_stream;
    char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';

    std::string compensate_zeros_right = get_string_compensate(std::abs(velocity_commands_.at(0)));
    std::string compensate_zeros_left = get_string_compensate(std::abs(velocity_commands_.at(1)));
    
    // std::fixed 고정 소수점 표기법을 사용하도록 한다.
    // std::setprecision(2) 출력 스트림의 정밀도. 2로 설정되어 있으면 소수점 두자리까지만 출력된다.
    message_stream << std::fixed << std::setprecision(2) 
    << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << ","
    << "l" << left_wheel_sign  << compensate_zeros_left  <<  std::abs(velocity_commands_.at(1)) << ",";



    // RCLCPP_INFO_STREAM(rclcpp::get_logger("DocbotInterface"), "write Serial - " << message_stream.str());
    try
    {
        arduino_.Write(message_stream.str());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("DocbotInterface"),
            "메시지를 보내는 중에 문제가 발생했습니다." << message_stream.str() << " port " << port_);

        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

std::string DocbotInterface::get_string_compensate(double velocity)
{
    if(velocity < 10) return "0";
    return "";
}

} // namespace bumperbot_firmware


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(docbot_firmware::DocbotInterface, hardware_interface::SystemInterface)