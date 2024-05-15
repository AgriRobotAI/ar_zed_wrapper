#ifndef ZED_STREAMER_NODE_
#define ZED_STREAMER_NODE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <sl/Camera.hpp>

#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <regex>
#include <string>
#include <iostream>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <ar_zed_msgs/msg/stereo_composite.hpp>
#include <ar_zed_msgs/msg/combined_camera_info.hpp>


using namespace std::chrono_literals;

class ZedStreamer : public rclcpp::Node
{
public:
  ZedStreamer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("zed_streamer_node", options)
  {
    initParameters();
    initCameras();
    getCameraInfoMsg();


    // publishing period for each camera
    int period_ms = static_cast<int>(periode_ * 1000 / NB_ZED_);

    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ZedStreamer::timer_callback, this));

    rclcpp::SensorDataQoS qos;
    qos.keep_last(5);
    qos.reliable();

    composite_pub_front_ = this->create_publisher<ar_zed_msgs::msg::StereoComposite>(
      "/stereo_composite/front",
      qos);

    composite_pub_rear_ = this->create_publisher<ar_zed_msgs::msg::StereoComposite>(
      "/stereo_composite/rear",
      qos);

    camera_info_pub_ = this->create_publisher<ar_zed_msgs::msg::CombinedCameraInfo>(
      "/combined_camera_info", qos);

  }

  ~ZedStreamer()
  {

    for (int z = 0; z < NB_ZED_; z++) {
      if (zeds_[z].isOpened()) {
        if (pool_[z].joinable()) {
          pool_[z].join();
        }
      }
    }

    for (int z = 0; z < NB_ZED_; z++) {
      zeds_[z].close();
    }
  }

private:
  void initParameters()
  {
    this->declare_parameter("publish_periode", 4.0);
    periode_ = this->get_parameter("publish_periode").as_double();

  }
  void initCameras();
  void timer_callback();

  void getCameraInfoMsg();
  void fillCamInfo(
    sl::CalibrationParameters zedParam,
    sl::Resolution resolution,
    sensor_msgs::msg::CameraInfo & leftCamInfoMsg,
    sensor_msgs::msg::CameraInfo & rightCamInfoMsg,
    const std::string & leftFrameId, const std::string & rightFrameId);

  sensor_msgs::msg::Image depthImageToMsg(
    sl::Mat & depth_map, rclcpp::Time t,
    std::string camera_name);
  rclcpp::Time slTimestampToROSTime(const sl::Timestamp & t);
  sensor_msgs::msg::Image imageToROSmsg(sl::Mat & img, rclcpp::Time t, std::string camera_name);

private:
  int NB_ZED_;
  std::vector<sl::Camera> zeds_;
  std::vector<sl::Camera>::iterator zeds_ptr_;
  std::vector<std::thread> pool_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::Publisher<ar_zed_msgs::msg::StereoComposite>> composite_pub_front_;
  std::shared_ptr<rclcpp::Publisher<ar_zed_msgs::msg::StereoComposite>> composite_pub_rear_;

  std::shared_ptr<rclcpp::Publisher<ar_zed_msgs::msg::CombinedCameraInfo>> camera_info_pub_;
  ar_zed_msgs::msg::CombinedCameraInfo combined_cam_info_;

  double periode_;

  // camera serial number to camera name
  std::map<unsigned int,
    std::string> camera_names_ = {{40969493, "camera_front"}, {47015842, "camera_rear"}};
};

void ZedStreamer::initCameras()
{
  sl::InitParameters initParameters;
  initParameters.camera_resolution = sl::RESOLUTION::HD1200;
  initParameters.depth_mode = sl::DEPTH_MODE::NEURAL;
  initParameters.camera_fps = 15;
  initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  initParameters.coordinate_units = sl::UNIT::MILLIMETER;
  initParameters.depth_minimum_distance = 300;
  initParameters.depth_maximum_distance = 10000;
  initParameters.sdk_verbose = true;
  initParameters.sdk_gpu_id = 0;
  initParameters.camera_image_flip = sl::FLIP_MODE::OFF;


  NB_ZED_ = sl::Camera::getDeviceList().size();

  if (NB_ZED_ == 0) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Cameras not found: " << NB_ZED_);
    exit(0);
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Try to open " << NB_ZED_ << " cameras with config: \n\tResolution: " << initParameters.camera_resolution << "\n\tfps: " << initParameters.camera_fps << "\n\tdepth mode: " <<
      initParameters.depth_mode);

  // Create the ZED camera
  zeds_ = std::vector<sl::Camera>(NB_ZED_);
  pool_ = std::vector<std::thread>(NB_ZED_);

  for (int z = 0; z < NB_ZED_; z++) {
    initParameters.input.setFromCameraID(z);
    // Open the camera
    sl::ERROR_CODE err = zeds_[z].open(initParameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "ZED [" << z << "] can not be opened, " << err);
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), NB_ZED_ << " ZEDs initialized. Ready for STREAMING");

  zeds_ptr_ = zeds_.begin();
}


void ZedStreamer::timer_callback()
{
  // get the camera name
  unsigned int serial_number = zeds_ptr_->getCameraInformation().serial_number;
  RCLCPP_DEBUG(this->get_logger(), "Camera serial number: %d", serial_number);

  std::string camera_name = camera_names_[serial_number];
  RCLCPP_DEBUG(this->get_logger(), "Camera name: %s", camera_name.c_str());

  std::shared_ptr<rclcpp::Publisher<ar_zed_msgs::msg::StereoComposite>> publisher;
  if (camera_name == "camera_front") {
    publisher = composite_pub_front_;
  } else if (camera_name == "camera_rear") {
    publisher = composite_pub_rear_;
  }


  sl::Mat image_right_n_1;
  sl::Mat image_left_n_1;
  sl::Timestamp timestamp_n_1;

  RCLCPP_DEBUG(this->get_logger(), "Grabbing images 1");
  if (zeds_ptr_->grab() == sl::ERROR_CODE::SUCCESS) {
    zeds_ptr_->retrieveImage(image_right_n_1, sl::VIEW::RIGHT);
    zeds_ptr_->retrieveImage(image_left_n_1, sl::VIEW::LEFT);
    timestamp_n_1 = zeds_ptr_->getTimestamp(sl::TIME_REFERENCE::IMAGE);
  }

  sl::Mat image_right_n;
  sl::Mat image_left_n;
  sl::Timestamp timestamp_n;
  sl::Mat depth_map;

  RCLCPP_DEBUG(this->get_logger(), "Grabbing images 2");
  if (zeds_ptr_->grab() == sl::ERROR_CODE::SUCCESS) {
    zeds_ptr_->retrieveImage(image_left_n, sl::VIEW::LEFT);
    zeds_ptr_->retrieveImage(image_right_n, sl::VIEW::RIGHT);
    timestamp_n = zeds_ptr_->getTimestamp(sl::TIME_REFERENCE::IMAGE);
    zeds_ptr_->retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
  }


  // create the message
  ar_zed_msgs::msg::StereoComposite msg;
  msg.right_n_1 = imageToROSmsg(image_right_n_1, slTimestampToROSTime(timestamp_n_1), camera_name);
  msg.right_n = imageToROSmsg(image_right_n, slTimestampToROSTime(timestamp_n), camera_name);
  msg.left_n_1 = imageToROSmsg(image_left_n_1, slTimestampToROSTime(timestamp_n_1), camera_name);
  msg.left_n = imageToROSmsg(image_left_n, slTimestampToROSTime(timestamp_n), camera_name);
  msg.capture_time_n_1 = slTimestampToROSTime(timestamp_n_1);
  msg.capture_time_n = slTimestampToROSTime(timestamp_n);
  msg.depth_n = depthImageToMsg(depth_map, slTimestampToROSTime(timestamp_n), camera_name);

  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing message of camera: " << camera_name);
  publisher->publish(msg);

  // increment the camera pointer
  zeds_ptr_++;
  if (zeds_ptr_ == zeds_.end()) {
    zeds_ptr_ = zeds_.begin();
  }


  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing camera info");
  camera_info_pub_->publish(combined_cam_info_);

}

sensor_msgs::msg::Image ZedStreamer::depthImageToMsg(
  sl::Mat & depth_map, rclcpp::Time t,
  std::string camera_name)
{
  // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
  sensor_msgs::msg::Image msg;
  msg.header.stamp = t;
  msg.header.frame_id = camera_name;
  msg.height = depth_map.getHeight();
  msg.width = depth_map.getWidth();

  int num = 1;     // for endianness detection
  msg.is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  msg.step = msg.width * sizeof(uint16_t);
  msg.encoding = sensor_msgs::image_encodings::MONO16;

  size_t size = msg.step * msg.height;
  msg.data.resize(size);

  uint16_t * data = reinterpret_cast<uint16_t *>(&msg.data[0]);

  int dataSize = msg.width * msg.height;
  sl::float1 * depthDataPtr = depth_map.getPtr<sl::float1>();

  for (int i = 0; i < dataSize; i++) {
    *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++)));     // in mm, rounded
  }

  return msg;
}

rclcpp::Time ZedStreamer::slTimestampToROSTime(const sl::Timestamp & t)
{
  rclcpp::Time ts(t.data_ns);
  return ts;
}

sensor_msgs::msg::Image ZedStreamer::imageToROSmsg(
  sl::Mat & img, rclcpp::Time t,
  std::string camera_name)
{
  sensor_msgs::msg::Image msg;

  msg.header.stamp = t;
  msg.header.frame_id = camera_name;
  msg.height = img.getHeight();
  msg.width = img.getWidth();

  int num = 1;     // for endianness detection
  msg.is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  msg.step = img.getStepBytes();

  size_t size = msg.step * msg.height;

  uint8_t * data_ptr = nullptr;

  sl::MAT_TYPE dataType = img.getDataType();

  switch (dataType) {
    case sl::MAT_TYPE::F32_C1:     /**< float 1 channel.*/
      msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C2:     /**< float 2 channels.*/
      msg.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float2>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C3:     /**< float 3 channels.*/
      msg.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float3>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C4:     /**< float 4 channels.*/
      msg.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float4>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C1:     /**< unsigned char 1 channel.*/
      msg.encoding = sensor_msgs::image_encodings::MONO8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar1>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C2:     /**< unsigned char 2 channels.*/
      msg.encoding = sensor_msgs::image_encodings::TYPE_8UC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar2>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C3:     /**< unsigned char 3 channels.*/
      msg.encoding = sensor_msgs::image_encodings::BGR8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar3>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C4:     /**< unsigned char 4 channels.*/
      msg.encoding = sensor_msgs::image_encodings::BGRA8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar4>());
      msg.data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;
  }

  return msg;
}


// function copied and simplified from ZED wrapper:
// https://github.com/stereolabs/zed-ros2-wrapper/blob/f88bb492fa01754365f0ac0d56e70fd29d55bbee/zed_components/src/zed_camera/src/zed_camera_component.cpp#L3148
void ZedStreamer::fillCamInfo(
  sl::CalibrationParameters zedParam,
  sl::Resolution resolution,
  sensor_msgs::msg::CameraInfo & leftCamInfoMsg,
  sensor_msgs::msg::CameraInfo & rightCamInfoMsg,
  const std::string & leftFrameId, const std::string & rightFrameId)
{
  RCLCPP_DEBUG(
    this->get_logger(), "param left fx: %f, fy: %f, cx: %f, cy: %f",
    zedParam.left_cam.fx, zedParam.left_cam.fy, zedParam.left_cam.cx, zedParam.left_cam.cy);
  RCLCPP_DEBUG(
    this->get_logger(), "param right fx: %f, fy: %f, cx: %f, cy: %f",
    zedParam.right_cam.fx, zedParam.right_cam.fy, zedParam.right_cam.cx, zedParam.right_cam.cy);

  float baseline = zedParam.getCameraBaseline();


  // ----> Distortion models
  // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
  // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3,
  // s4) distortion. Prism not currently used.

  // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4

  leftCamInfoMsg.distortion_model =
    sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
  rightCamInfoMsg.distortion_model =
    sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
  leftCamInfoMsg.d.resize(8);
  rightCamInfoMsg.d.resize(8);
  leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];        // k1
  leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];        // k2
  leftCamInfoMsg.d[2] = zedParam.left_cam.disto[2];        // p1
  leftCamInfoMsg.d[3] = zedParam.left_cam.disto[3];        // p2
  leftCamInfoMsg.d[4] = zedParam.left_cam.disto[4];        // k3
  leftCamInfoMsg.d[5] = zedParam.left_cam.disto[5];        // k4
  leftCamInfoMsg.d[6] = zedParam.left_cam.disto[6];        // k5
  leftCamInfoMsg.d[7] = zedParam.left_cam.disto[7];        // k6
  rightCamInfoMsg.d[0] = zedParam.right_cam.disto[0];      // k1
  rightCamInfoMsg.d[1] = zedParam.right_cam.disto[1];      // k2
  rightCamInfoMsg.d[2] = zedParam.right_cam.disto[2];      // p1
  rightCamInfoMsg.d[3] = zedParam.right_cam.disto[3];      // p2
  rightCamInfoMsg.d[4] = zedParam.right_cam.disto[4];      // k3
  rightCamInfoMsg.d[5] = zedParam.right_cam.disto[5];      // k4
  rightCamInfoMsg.d[6] = zedParam.right_cam.disto[6];      // k5
  rightCamInfoMsg.d[7] = zedParam.right_cam.disto[7];      // k6


  leftCamInfoMsg.k.fill(0.0);
  rightCamInfoMsg.k.fill(0.0);
  leftCamInfoMsg.k[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg.k[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg.k[4] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg.k[5] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg.k[8] = 1.0;
  rightCamInfoMsg.k[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg.k[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg.k[4] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg.k[5] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg.k[8] = 1.0;
  leftCamInfoMsg.r.fill(0.0);
  rightCamInfoMsg.r.fill(0.0);

  for (size_t i = 0; i < 3; i++) {
    // identity
    rightCamInfoMsg.r[i + i * 3] = 1;
    leftCamInfoMsg.r[i + i * 3] = 1;
  }

  leftCamInfoMsg.p.fill(0.0);
  rightCamInfoMsg.p.fill(0.0);
  leftCamInfoMsg.p[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg.p[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg.p[5] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg.p[6] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg.p[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  rightCamInfoMsg.p[3] =
    static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
  rightCamInfoMsg.p[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg.p[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg.p[5] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg.p[6] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg.p[10] = 1.0;


  leftCamInfoMsg.width = rightCamInfoMsg.width =
    static_cast<uint32_t>(resolution.width);
  leftCamInfoMsg.height = rightCamInfoMsg.height =
    static_cast<uint32_t>(resolution.width);

  leftCamInfoMsg.header.frame_id = leftFrameId;
  rightCamInfoMsg.header.frame_id = rightFrameId;
}

// get left and right camera parameters of all zed cameras and push them to private variable
void ZedStreamer::getCameraInfoMsg()
{
  for (int i = 0; i < NB_ZED_; i++) {
    sl::CalibrationParameters zedParam =
      zeds_[i].getCameraInformation().camera_configuration.calibration_parameters;
    sl::Resolution resolution = zeds_[i].getCameraInformation().camera_configuration.resolution;

    sensor_msgs::msg::CameraInfo leftCamInfoMsg;
    sensor_msgs::msg::CameraInfo rightCamInfoMsg;

    fillCamInfo(
      zedParam,
      resolution,
      leftCamInfoMsg,
      rightCamInfoMsg,
      "left", "right");


    // select camera front or rear
    unsigned int serial_number = zeds_[i].getCameraInformation().serial_number;
    std::string camera_name = camera_names_[serial_number];
    if (camera_name == "camera_front") {
      combined_cam_info_.front_left = leftCamInfoMsg;
      combined_cam_info_.front_right = rightCamInfoMsg;
      RCLCPP_INFO(this->get_logger(), "Camera name: %s, assigned config", camera_name.c_str());
    } else if (camera_name == "camera_rear") {
      combined_cam_info_.rear_left = leftCamInfoMsg;
      combined_cam_info_.rear_right = rightCamInfoMsg;
      RCLCPP_INFO(this->get_logger(), "Camera name: %s, assigned config", camera_name.c_str());
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ZedStreamer)

#endif // ZED_STREAMER_NODE__
