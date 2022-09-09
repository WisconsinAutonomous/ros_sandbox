#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageProcessorNode : public rclcpp::Node
{
	public:
		ImageProcessorNode() : Node("image_processor_node"), count_(0), clock_()
		{
			image_sub = this->create_subscription<sensor_msgs::msg::Image>("image/original", 10, std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));
            
            image_pub = this->create_publisher<sensor_msgs::msg::Image>("image/grayscale", 10);
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

            // convert Image msg to CvImagePtr
            cv_bridge::CvImagePtr colorPtr;
            colorPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // get grayscale image
            cv::Mat colorMat = colorPtr->image;
            cv::Mat grayMat;
            cv::cvtColor(colorMat, grayMat, CV_BGR2GRAY);

            // convert cv::Mat to CvImage
            std_msgs::msg::Header header; // empty header
            header.stamp = clock_.now();
            cv_bridge::CvImage grayPtr;
            grayPtr = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, grayMat);

            // publish
            image_pub->publish(*grayPtr.toImageMsg());
		}

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
		size_t count_;
		rclcpp::Clock clock_;

};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageProcessorNode>());
	rclcpp::shutdown();
	return 0;
}
