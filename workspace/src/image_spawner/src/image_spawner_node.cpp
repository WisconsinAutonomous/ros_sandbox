#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::literals::chrono_literals;
class ImageSpawnerNode : public rclcpp::Node
{
	public:
		ImageSpawnerNode() : Node("image_spawner_node"), count_(0), clock_()
		{
			image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
			timer_ = this->create_wall_timer(500ms, std::bind(&ImageSpawnerNode::timer_callback, this));
			//clock_ = rclcpp::Clock::Clock();
		}
	private:
		void timer_callback()
		{
			cv_bridge::CvImagePtr cv_ptr;
			cv::Mat img(cv::Size(1280, 720), CV_8UC3);
			cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

			sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();



			msg->header.stamp = clock_.now();
			image_publisher->publish(*msg.get());
		}

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
		size_t count_;
		rclcpp::Clock clock_;

};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageSpawnerNode>());
	rclcpp::shutdown();
	return 0;
}


