#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"


class Ros2Node : public rclcpp::Node
{
public:
	Ros2Node();

	inline cv_bridge::CvImagePtr getRgbImage() {return m_rgbImage;}
	inline cv_bridge::CvImagePtr getIrImage() {return m_irImage;}

	void captureImages();
private:

	void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);
	void irCallback(const sensor_msgs::msg::Image::SharedPtr msg);


	cv_bridge::CvImagePtr m_rgbImage;
	cv_bridge::CvImagePtr m_irImage;

	int m_imageCounter;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_rgbSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_irSubscriber;
};