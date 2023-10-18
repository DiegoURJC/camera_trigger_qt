#include "camera_trigger_qt/ros2node.hpp"
#include <opencv2/imgcodecs.hpp>


using std::placeholders::_1;

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node"),
    m_rgbImage(nullptr),
    m_irImage(nullptr),
    m_imageCounter(1)
{
  m_rgbSubscriber = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 30, std::bind(&Ros2Node::rgbCallback, this, _1));
  m_irSubscriber = create_subscription<sensor_msgs::msg::Image>("/camera/infra1/image_rect_raw", 30, std::bind(&Ros2Node::irCallback, this, _1));
  m_odometrySubscriber = create_subscription<px4_msgs::msg::VehicleOdometry>
    ("/fmu/vehicle_odometry/out", 100, std::bind(&Ros2Node::odom_callback, this, _1));
}

void Ros2Node::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    m_rgbImage = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
  }
}

void Ros2Node::irCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    m_irImage = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception %s", e.what());
  }
}

void Ros2Node::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  m_position = {msg->x, msg->y, msg->z};
  const std::array<float, 4> q = msg->q;

  quaternion2euler(q);
}



void Ros2Node::quaternion2euler(const std::array<float, 4> &q) noexcept
{
  constexpr int32_t W = 0;
  constexpr int32_t X = 1;
  constexpr int32_t Y = 2;
  constexpr int32_t Z = 3;
  constexpr int32_t ROLL = 0;
  constexpr int32_t PITCH = 1;
  constexpr int32_t YAW = 2; 

  // Conversion to roll, pitch, yaw
  m_odom[ROLL] = std::atan2(2 * (q[W] * q[X] + q[Y] * q[Z]), 
                         1 - 2 * (q[X] * q[X] + q[Y] * q[Y]));
  m_odom[PITCH] = std::asin(2 * (q[W] * q[Y] - q[Z] * q[X]));
  m_odom[YAW] = std::atan2(2 * (q[W] * q[Z] + q[X] * q[Y]), 
                         1 - 2 * (q[Y] * q[Y] + q[Z] * q[Z]));
}

void Ros2Node::captureImages()
{
  RCLCPP_INFO(get_logger(), "CAPTURING RGB & IR IMAGES...");

  if((m_rgbImage != nullptr) && (m_irImage != nullptr))
  {
    cv::String rgbImageStr = "rgb_image_" + std::to_string(m_imageCounter) + ".png";
    cv::String irImageStr = "ir_image_" + std::to_string(m_imageCounter) + ".png";

    std::cout << rgbImageStr << std::endl;
    std::cout << irImageStr << std::endl;

    cv::Mat rgbImage = m_rgbImage->image;
    cv::Mat irImage = m_irImage->image;

    cv::cvtColor(rgbImage, rgbImage, cv::COLOR_BGR2RGB);

    cv::imwrite(rgbImageStr, rgbImage);
    cv::imwrite(irImageStr, irImage);

    ++m_imageCounter;

    m_rgbImage = nullptr;
    m_irImage = nullptr;
  }
}
