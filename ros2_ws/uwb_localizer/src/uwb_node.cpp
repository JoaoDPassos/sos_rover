#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>

/*
    Ultra wide-band (UWB) node class for DWM1001 UWB modules.
    Args:
        - name: Name of UWB Node.
        - port: Port at which UWB Node is connected, visible in linux
        - baud: baud rate at which to maintain communications with UWB Node

    Member Variables:
        - serial_: object from serial-ros2 library which manages all serial communications
        - pub_: ros2 publisher which enables other packages to recieve data being
        transmitted from the UWB Node
        - timer_: every 100 milliseconds the timer reads incoming serial data
*/

class UwbNode : public rclcpp::Node {
public:
    UwbNode(const std::string &name, const std::string &port, int baud) 
        : Node(name), serial_(port, baud, serial::Timeout::simpleTimeout(1000)) {

        // Publisher for relavant packages to read node data:
        pub_ = this->create_publisher<std_msgs::msg::String>("uwb_data", 10);
        
        // Read incoming Serial every 0.1 seconds
    	open_serial();
    }

private:
    void open_serial() {
	if (!serial_.isOpen()) {
            try {
            	serial_.open();
                RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", serial_.getPort().c_str());
            } catch (serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
                return;
            }
	}
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&UwbNode::read_serial, this));
    }

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

    auto node = std::make_shared<UwbNode>(
        "uwb_node",       // node name
        "/dev/ttyACM0",   // port
        115200            // baud rate
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

