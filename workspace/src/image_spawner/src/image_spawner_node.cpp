#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/Image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include "std_msgs/msg/header.hpp"

class ImageSpawnerNode : public rclcpp::Node
{
	public:
		ImageSpawnerNode() : Node("image_spawner_node"), count_(0)
		{
			image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
			timer_ = this->create_wall_timer(500ms, std::bind(&ImageSpawnerNode::timer_callback, this));
		}
	private:
		void timer_callback()
		{
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat img(cv::Size(1280, 720), CV_8UC3);
			cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

			sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std::msgs::msg::Header(), "bgr8", img).toImageMsg();

			publisher_->publish(*msg.get());
		}

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
		size_t count_;

};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageSpawnerNode>());
	rclcpp::shutdown();
	return 0;
}


