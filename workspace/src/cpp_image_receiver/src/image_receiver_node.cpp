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
		ImageReceiverNode() : Node("image_receiver_node"), count_(0), clock_()
		{
			image_sub = this->create_subscription<sensor_msgs::msg::Image>("image/grayscale", 10, std::bind(&ImageReceiverNode::image_callback, this, std::placeholders::_1));
		}
	private:
		void image_callback (const sensor_msgs::msg::Image::SharedPtr msg) 
		{

			rclcpp::Time cur_time = clock_.now();
			const auto secs = cur_time.nanoseconds()/1000000000UL - msg->header.stamp.sec;
			const auto nsecs = cur_time.nanoseconds()%1000000000UL - msg->header.stamp.nanosec;

			std::stringstream ss;
			ss << "Time diff: " << secs << "." << nsecs << "s";
			RCLCPP_INFO(this->get_logger(), ss.str().c_str());
		}

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
		size_t count_;
		rclcpp::Clock clock_;

};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageReceiverNode>());
	rclcpp::shutdown();
	return 0;
}
