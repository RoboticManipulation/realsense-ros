#include <chrono>
#include <functional>
#include <memory>
#include <inttypes.h>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
//using std::placeholders::_1;

class RepublishCameraInfo : public rclcpp::Node
{
  public:
    RepublishCameraInfo()
    : Node("republish_cam_info")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info_factory", 10, std::bind(&RepublishCameraInfo::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msgs) const
    {
      //Output of original message
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", camera_info_msgs.header.frame_id.c_str());

      //Change the header
      //auto message = sensor_msgs::msg::CameraInfo();

      auto message = *camera_info_msgs;
      // frame_id of image and depth_image
      // message.header.frame_id = "camera_l515_color_optical_frame";
      // frame_id of points


      //std::array<double, 5> d = {0.10119861778233893, -0.1954721576865799, 0.0012448486976424538, 0.00191865834729546, 0.0};
      double d_arr[5] = {0.10119861778233893, -0.1954721576865799, 0.0012448486976424538, 0.00191865834729546, 0.0};
      std::vector<double> d_vec(d_arr, d_arr+5);
      message.d = d_vec;

      std::array<double, 9> k_std_arr = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 1.0};
      //double k_arr[9] = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 1.0};
      //std::vector<double> k_vec(k_arr, k_arr+9);
      message.k = k_std_arr;

      std::array<double, 12> p_std_arr = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 0.0, 1.0, 0.0};
      //double p_arr[12] = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 0.0, 1.0, 0.0};
      //std::vector<double> p_vec(p_arr, p_arr+12);
      message.p = p_std_arr;

      //Output of new message
      //RCLCPP_INFO(this->get_logger(), "I publish: '%s'", message.header.frame_id.c_str());

      //Publish the new message
      publisher_->publish(message);


      /*
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", camera_info_msgs.header.frame_id.c_str());
      RCLCPP_INFO(this->get_logger(), "I heard: ");

      pcl::PointCloud<pcl::PointXYZ> camera_info;
      pcl::fromROSMsg(camera_info_msgs, camera_info);     

      BOOST_FOREACH(const pcl::PointXYZ& pt, camera_info.points)
      {          
        std::cout  << "x: " << pt.x <<"\n";
        std::cout  << "y: " << pt.y <<"\n";
        std::cout  << "z: " << pt.z <<"\n";
        std::cout << "---------" << "\n";
      }
      */
    }
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepublishCameraInfo>());
  rclcpp::shutdown();
  return 0;
}