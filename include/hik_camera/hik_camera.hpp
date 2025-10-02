#ifndef HIK_CAMERA_HPP
#define HIK_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <atomic>
#include <thread>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class HikCamera : public rclcpp::Node
{
public:
    HikCamera();
    ~HikCamera();
    
    bool initialize();
    void startStreaming();
    void stopStreaming();
    
private:
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    void initParameters();
    bool connectCamera();
    bool reconnectCamera(); // 新增：断线重连功能
    void disconnectCamera();
    void imageCaptureThread();
    void publishImage(const cv::Mat& image);
    
    // 海康SDK句柄
    void* camera_handle_;
    MV_CC_DEVICE_INFO_LIST device_list_;
    
    // ROS2组件
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 线程控制
    std::atomic<bool> streaming_;
    std::atomic<bool> reconnect_flag_; // 新增：重连标志
    std::thread capture_thread_;
    std::thread reconnect_thread_; // 新增：重连线程
    
    // 相机参数
    int exposure_time_;
    double gain_;
    double frame_rate_;
    std::string pixel_format_;
    std::string camera_ip_;
    std::string camera_serial_; // 新增：相机序列号支持
    std::string image_topic_; // 新增：可配置的Topic名称
};

#endif