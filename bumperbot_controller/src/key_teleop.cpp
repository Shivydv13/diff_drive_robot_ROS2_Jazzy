#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>  // Required for fcntl()
#include <iostream>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/input_key/cmd_vel_stamped", 10);
        RCLCPP_INFO(this->get_logger(), "Use W/A/S/D to move, Q to stop.");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&KeyboardTeleop::publishCommand, this));
        setNonBlocking();
    }

    ~KeyboardTeleop()
    {
        resetTerminal();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    char key_ = 0;

    void setNonBlocking()
    {
        tcgetattr(STDIN_FILENO, &original_termios_);
        struct termios new_settings = original_termios_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void resetTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }

    void publishCommand()
    {
        geometry_msgs::msg::TwistStamped msg;
        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0)
        {
            key_ = tolower(ch);
        }

        switch (key_)
        {
        case 'w':
            msg.twist.linear.x = 0.5;
            msg.twist.angular.z = 0.0;
            break;
        case 's':
            msg.twist.linear.x = -0.5;
            msg.twist.angular.z = 0.0;
            break;
        case 'a':
            msg.twist.linear.x = 0.0;
            msg.twist.angular.z = 1.0;
            break;
        case 'd':
            msg.twist.linear.x = 0.0;
            msg.twist.angular.z = -1.0;
            break;
        case 'q':
            msg.twist.linear.x = 0.0;
            msg.twist.angular.z = 0.0;
            break;
        default:
            return; // Skip publishing if no valid key
        }

        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";  // Set this to your robot's frame

        publisher_->publish(msg);
    }

    struct termios original_termios_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
