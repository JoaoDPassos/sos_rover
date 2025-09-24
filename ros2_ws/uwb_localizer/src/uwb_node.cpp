#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>

class UwbNode : public rclcpp::Node {
public:
    UwbNode() : Node("uwb_node") {
        pub_ = this->create_publisher<std_msgs::msg::String>("uwb_data", 10);

        try {
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(115200);
            serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&UwbNode::read_serial, this));
    }

private:
    void read_serial() {
        if (serial_.isOpen()) {
            std::string line = serial_.readline();
            auto msg = std_msgs::msg::String();
            msg.data = line;
            pub_->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UwbNode>());
    rclcpp::shutdown();
    return 0;
}
