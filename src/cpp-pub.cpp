#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//시간 +문자 조합 사용
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

//최소 출력 노드
class MinimalPublisher : public rclcpp::Node
{
  public:
    //노드 생성 및 초기화 설정
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      //토픽 메시지 출력 및 크기 설정
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // 타이머이용 5초마다 콜백을 바인딩(노드와 콜백함수를 묶는다.
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    // 타이머_콜백 함수
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      //위 메시지를 info로 로그 출력
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // 노드에 전달
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
