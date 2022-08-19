#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageReceiverNode : public rclcpp::Node
{
	public:
		ImageReceiverNode() : Node("image_receiver_node"), count_(0)
		{
			image_subscription = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&ImageReceiverNode::topic_callback, this, std::placeholders::_1));
		}
	private:
		void image_received_callback (const sensor_msgs::msg::Image::SharedPtr msg) const
		{
			RCLPP_INFO(this->get_logger(), "Image received!");
		}

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;

};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageReceiverNode>());
	rclcpp::shutdown();
	return 0;
}
