#include "hik_camera/hik_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using std::placeholders::_1;

HikCamera::HikCamera() : Node("hik_camera"), camera_handle_(nullptr), streaming_(false), reconnect_flag_(false)
{
    // 声明参数
    this->declare_parameter<std::string>("camera_ip", "192.168.1.100");
    this->declare_parameter<std::string>("camera_serial", "");
    this->declare_parameter<int>("exposure_time", 10000);
    this->declare_parameter<double>("gain", 1.0);
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<std::string>("pixel_format", "BGR8");
    this->declare_parameter<std::string>("image_topic", "image_raw"); // 新增：可配置Topic
    
    // 获取参数
    camera_ip_ = this->get_parameter("camera_ip").as_string();
    camera_serial_ = this->get_parameter("camera_serial").as_string();
    exposure_time_ = this->get_parameter("exposure_time").as_int();
    gain_ = this->get_parameter("gain").as_double();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    pixel_format_ = this->get_parameter("pixel_format").as_string();
    image_topic_ = this->get_parameter("image_topic").as_string(); // 新增：获取Topic名称
    
    // 创建发布者
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    
    // 参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCamera::parametersCallback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "HikCamera node initialized");
}

HikCamera::~HikCamera()
{
    stopStreaming();
    disconnectCamera();
}

bool HikCamera::initialize()
{
    // 初始化SDK
    int nRet = MV_CC_Initialize();
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "MVS SDK initialization failed! Error code: %x", nRet);
        return false;
    }
    
    // 枚举设备
    unsigned int nAccessType = MV_GIGE_DEVICE | MV_USB_DEVICE;
    nRet = MV_CC_EnumDevices(nAccessType, &device_list_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Enum devices failed! Error code: %x", nRet);
        return false;
    }
    
    if (device_list_.nDeviceNum == 0) {
        RCLCPP_WARN(this->get_logger(), "No camera found, running in simulation mode");
        return true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Found %d cameras", device_list_.nDeviceNum);
    return connectCamera();
}

bool HikCamera::connectCamera()
{
    if (device_list_.nDeviceNum == 0) {
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode - no physical camera connected");
        return true;
    }
    
    // 根据IP地址或序列号选择相机
    int selected_index = 0;
    bool found = false;
    
    for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = device_list_.pDeviceInfo[i];
        
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            // 千兆网相机
            char* ip = pDeviceInfo->SpecialInfo.stGigEInfo.chCurrentIp;
            char* serial = pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber;
            
            if ((!camera_ip_.empty() && std::string(ip) == camera_ip_) ||
                (!camera_serial_.empty() && std::string(serial) == camera_serial_)) {
                selected_index = i;
                found = true;
                RCLCPP_INFO(this->get_logger(), "Found camera with IP: %s, Serial: %s", ip, serial);
                break;
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            // USB相机
            char* serial = pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            
            if (!camera_serial_.empty() && std::string(serial) == camera_serial_) {
                selected_index = i;
                found = true;
                RCLCPP_INFO(this->get_logger(), "Found USB camera with Serial: %s", serial);
                break;
            }
        }
    }
    
    if (!camera_ip_.empty() || !camera_serial_.empty()) {
        if (!found) {
            RCLCPP_ERROR(this->get_logger(), "No camera found with specified IP or serial number");
            return false;
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "No specific camera specified, using first available camera");
    }
    
    // 连接选择的相机
    int nRet = MV_CC_CreateHandle(&camera_handle_, device_list_.pDeviceInfo[selected_index]);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Create handle failed! Error code: %x", nRet);
        return false;
    }
    
    nRet = MV_CC_OpenDevice(camera_handle_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Open device failed! Error code: %x", nRet);
        return false;
    }
    
    // 设置相机参数
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
    
    // 根据像素格式设置相机
    if (pixel_format_ == "BGR8") {
        MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    } else if (pixel_format_ == "RGB8") {
        MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
    } else if (pixel_format_ == "Mono8") {
        MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_Mono8);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %s, using default BGR8", pixel_format_.c_str());
        MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    }
    
    RCLCPP_INFO(this->get_logger(), "Camera connected successfully");
    return true;
}

bool HikCamera::reconnectCamera()
{
    RCLCPP_WARN(this->get_logger(), "Attempting to reconnect camera...");
    disconnectCamera();
    
    // 尝试重新枚举设备
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Re-enum devices failed! Error code: %x", nRet);
        return false;
    }
    
    if (device_list_.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No cameras found during reconnection attempt");
        return false;
    }
    
    return connectCamera();
}

void HikCamera::startStreaming()
{
    if (!streaming_) {
        streaming_ = true;
        reconnect_flag_ = false;
        
        if (device_list_.nDeviceNum > 0) {
            capture_thread_ = std::thread(&HikCamera::imageCaptureThread, this);
            RCLCPP_INFO(this->get_logger(), "Image streaming started");
        } else {
            RCLCPP_INFO(this->get_logger(), "Running in simulation mode - no image streaming");
        }
    }
}

void HikCamera::stopStreaming()
{
    streaming_ = false;
    reconnect_flag_ = false;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
    
    RCLCPP_INFO(this->get_logger(), "Image streaming stopped");
}

void HikCamera::imageCaptureThread()
{
    if (device_list_.nDeviceNum == 0) {
        // 模拟模式：生成测试图像
        while (streaming_) {
            cv::Mat test_image(480, 640, CV_8UC3);
            test_image = cv::Scalar(100, 100, 255); // 蓝色背景
            cv::putText(test_image, "HIK Camera Simulation Mode", 
                       cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                       cv::Scalar(255, 255, 255), 2);
            publishImage(test_image);
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30fps
        }
        return;
    }
    
    // 开始取流
    int nRet = MV_CC_StartGrabbing(camera_handle_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Start grabbing failed! Error code: %x", nRet);
        return;
    }
    
    MV_FRAME_OUT stImageInfo;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    
    int consecutive_errors = 0;
    const int max_consecutive_errors = 5; // 最大连续错误次数
    
    while (streaming_) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);
        if (nRet == MV_OK) {
            consecutive_errors = 0; // 重置错误计数
            
            // 转换图像数据
            cv::Mat image;
            if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                image = cv::Mat(stImageInfo.stFrameInfo.nHeight, 
                               stImageInfo.stFrameInfo.nWidth, 
                               CV_8UC3, stImageInfo.pBufAddr);
            } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
                image = cv::Mat(stImageInfo.stFrameInfo.nHeight, 
                               stImageInfo.stFrameInfo.nWidth, 
                               CV_8UC3, stImageInfo.pBufAddr);
                cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8) {
                image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                               stImageInfo.stFrameInfo.nWidth,
                               CV_8UC1, stImageInfo.pBufAddr);
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %d", 
                           stImageInfo.stFrameInfo.enPixelType);
                image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                               stImageInfo.stFrameInfo.nWidth,
                               CV_8UC1, stImageInfo.pBufAddr);
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            }
            
            if (!image.empty()) {
                publishImage(image);
            }
            
            MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
        } else {
            consecutive_errors++;
            RCLCPP_WARN(this->get_logger(), "Get image buffer failed! Error code: %x, consecutive errors: %d", nRet, consecutive_errors);
            
            // 检查是否达到最大错误次数，触发重连
            if (consecutive_errors >= max_consecutive_errors && !reconnect_flag_) {
                reconnect_flag_ = true;
                RCLCPP_ERROR(this->get_logger(), "Too many consecutive errors, attempting to reconnect...");
                
                // 在单独的线程中尝试重连
                if (reconnect_thread_.joinable()) {
                    reconnect_thread_.join();
                }
                reconnect_thread_ = std::thread([this]() {
                    if (reconnectCamera()) {
                        RCLCPP_INFO(this->get_logger(), "Reconnection successful");
                        reconnect_flag_ = false;
                        consecutive_errors = 0;
                        
                        // 重新开始取流
                        int nRet = MV_CC_StartGrabbing(camera_handle_);
                        if (nRet != MV_OK) {
                            RCLCPP_ERROR(this->get_logger(), "Restart grabbing failed after reconnection! Error code: %x", nRet);
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Reconnection failed");
                    }
                });
            }
            
            // 短暂休眠后继续尝试
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    MV_CC_StopGrabbing(camera_handle_);
}

void HikCamera::publishImage(const cv::Mat& image)
{
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_optical_frame";
    image_pub_->publish(*msg);
}

rcl_interfaces::msg::SetParametersResult HikCamera::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    
    for (const auto& param : parameters) {
        if (param.get_name() == "exposure_time") {
            exposure_time_ = param.as_int();
            if (camera_handle_) {
                MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
            }
            RCLCPP_INFO(this->get_logger(), "Exposure time set to: %d", exposure_time_);
        } else if (param.get_name() == "gain") {
            gain_ = param.as_double();
            if (camera_handle_) {
                MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
            }
            RCLCPP_INFO(this->get_logger(), "Gain set to: %.2f", gain_);
        } else if (param.get_name() == "frame_rate") {
            frame_rate_ = param.as_double();
            if (camera_handle_) {
                MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
            }
            RCLCPP_INFO(this->get_logger(), "Frame rate set to: %.2f", frame_rate_);
        } else if (param.get_name() == "pixel_format") {
            pixel_format_ = param.as_string();
            if (camera_handle_) {
                if (pixel_format_ == "BGR8") {
                    MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
                } else if (pixel_format_ == "RGB8") {
                    MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
                } else if (pixel_format_ == "Mono8") {
                    MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_Mono8);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %s", pixel_format_.c_str());
                }
            }
            RCLCPP_INFO(this->get_logger(), "Pixel format set to: %s", pixel_format_.c_str());
        } else if (param.get_name() == "image_topic") {
            image_topic_ = param.as_string();
            // 需要重新创建发布者
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
            RCLCPP_INFO(this->get_logger(), "Image topic set to: %s", image_topic_.c_str());
        }
    }
    
    return result;
}

void HikCamera::disconnectCamera()
{
    if (camera_handle_) {
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    MV_CC_Finalize();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto camera_node = std::make_shared<HikCamera>();
    
    if (camera_node->initialize()) {
        camera_node->startStreaming();
        rclcpp::spin(camera_node);
        camera_node->stopStreaming();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Camera initialization failed");
    }
    
    rclcpp::shutdown();
    return 0;
}