#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>


class Ros2Node : public rclcpp::Node
{
public:
	Ros2Node();

	inline cv_bridge::CvImagePtr getRgbImage() {return m_rgbImage;}
	inline cv_bridge::CvImagePtr getIrImage() {return m_irImage;}
	inline std::array<double, 3> getPosition() {return m_position;}
	inline std::array<double, 3> getOdometry() {return m_odom;}

	void captureImages();
private:

	void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);
	void irCallback(const sensor_msgs::msg::Image::SharedPtr msg);
	void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

	void quaternion2euler(const std::array<float, 4> &q) noexcept;


	cv_bridge::CvImagePtr m_rgbImage;
	cv_bridge::CvImagePtr m_irImage;

	int m_imageCounter;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_rgbSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_irSubscriber;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr m_odometrySubscriber;

	std::array<double, 3> m_position;
	std::array<double, 3> m_odom;
};