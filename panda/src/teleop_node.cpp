#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <set>

class TeleopTwistStamped : public rclcpp::Node {
public:
  TeleopTwistStamped()
  : Node("teleop_twiststamped_node") {
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_cont/cmd_vel", 10);

    enableTerminalRawMode();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TeleopTwistStamped::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Use W/A/S/D keys for motion, Q to quit.");
  }

  ~TeleopTwistStamped() {
    disableTerminalRawMode();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  struct termios original_term_;

  void enableTerminalRawMode() {
    tcgetattr(STDIN_FILENO, &original_term_);
    struct termios raw = original_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  }

  void disableTerminalRawMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_term_);
  }

  void timerCallback() {
    std::set<char> keys_pressed;
    char c;

    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == 'q') {
        rclcpp::shutdown();
        return;
      }
      keys_pressed.insert(tolower(c));
    }

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";

    // Linear movement
    if (keys_pressed.count('w'))
      twist_msg.twist.linear.x = 0.5;
    if (keys_pressed.count('s'))
      twist_msg.twist.linear.x = -0.5;

    // Angular movement
    if (keys_pressed.count('a'))
      twist_msg.twist.angular.z = 0.5;
    if (keys_pressed.count('d'))
      twist_msg.twist.angular.z = -0.5;

    // Only publish if there's movement
    if (twist_msg.twist.linear.x != 0.0 || twist_msg.twist.angular.z != 0.0)
      pub_->publish(twist_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopTwistStamped>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
