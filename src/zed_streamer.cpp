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
#include <sensor_msgs/image_encodings.hpp>


void depthImageToROSmsg(
  sl::Mat & img,
  std::string frameId,
  rclcpp::Time t,
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub)
{

  // OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
  std::unique_ptr<sensor_msgs::msg::Image> msg = std::make_unique<sensor_msgs::msg::Image>();

  msg->header.stamp = t;
  msg->header.frame_id = frameId;
  msg->height = img.getHeight();
  msg->width = img.getWidth();

  int num = 1;  // for endianness detection
  msg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  msg->step = msg->width * sizeof(uint16_t);
  msg->encoding = sensor_msgs::image_encodings::MONO16;

  size_t size = msg->step * msg->height;
  msg->data.resize(size);

  uint16_t * data = reinterpret_cast<uint16_t *>(&msg->data[0]);

  int dataSize = msg->width * msg->height;
  sl::float1 * depthDataPtr = img.getPtr<sl::float1>();

  for (int i = 0; i < dataSize; i++) {
    *(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) /** 1000*/));  // in mm, rounded
    // NOTE(Walter) You set 
    // initParameters.coordinate_units = sl::UNIT::MILLIMETER;
    // initParameters.depth_minimum_distance = 300;
    // initParameters.depth_maximum_distance = 10000;
    // hence you must not multiply by 1000, the depth map is already in millimeters
  }

  pub->publish(std::move(msg));
}

void imageToROSmsg(
  sl::Mat & img,
  std::string frameId,
  rclcpp::Time t,
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub)
{
  std::unique_ptr<sensor_msgs::msg::Image> msg = std::make_unique<sensor_msgs::msg::Image>();

  msg->header.stamp = t;
  msg->header.frame_id = frameId;
  msg->height = img.getHeight();
  msg->width = img.getWidth();

  int num = 1;  // for endianness detection
  msg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

  msg->step = img.getStepBytes();

  size_t size = msg->step * msg->height;

  uint8_t * data_ptr = nullptr;

  sl::MAT_TYPE dataType = img.getDataType();

  switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float2>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float3>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float4>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
      msg->encoding = sensor_msgs::image_encodings::MONO8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar1>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
      msg->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar2>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
      msg->encoding = sensor_msgs::image_encodings::BGR8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar3>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
      msg->encoding = sensor_msgs::image_encodings::BGRA8;
      data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar4>());
      msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
      break;
  }

  pub->publish(std::move(*msg));
}


using namespace std::chrono_literals;

class ZedStreamer : public rclcpp::Node
{
public:
  ZedStreamer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("zed_streamer_node", options)
  {
    initCameras();

    timer_ =
      this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ZedStreamer::timer_callback, this));

    rclcpp::SensorDataQoS qos;
    qos.keep_last(5);
    //qos.best_effort(); 
    // NOTE(Walter): I recommend using the default settings (RELIABLE). 
    // Best effort causes many communication issues.
    // A short history is instead recommended

    zed_1_depth_ = this->create_publisher<sensor_msgs::msg::Image>("zed_1/depth", qos);
    zed_1_image_right_ = this->create_publisher<sensor_msgs::msg::Image>("zed_1/image_left", qos);
    zed_1_image_left_ = this->create_publisher<sensor_msgs::msg::Image>("zed_1/image_right", qos);
    zed_2_depth_ = this->create_publisher<sensor_msgs::msg::Image>("zed_2/depth", qos);
    zed_2_image_right_ = this->create_publisher<sensor_msgs::msg::Image>("zed_2/image_left", qos);
    zed_2_image_left_ = this->create_publisher<sensor_msgs::msg::Image>("zed_2/image_right", qos);
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
  void initCameras();
  void timer_callback();

private:
  int NB_ZED_;
  std::vector<sl::Camera> zeds_;
  std::vector<std::thread> pool_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> zed_1_depth_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> zed_1_image_right_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> zed_1_image_left_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> zed_2_depth_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> zed_2_image_right_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> zed_2_image_left_;
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
}

void ZedStreamer::timer_callback()
{
  RCLCPP_ERROR_STREAM(this->get_logger(), "Grabbing frame");
  {
    sl::Mat image_right;
    sl::Mat image_left;
    sl::Mat depth_map;

    if (zeds_[0].grab() == sl::ERROR_CODE::SUCCESS) {
      zeds_[0].retrieveImage(image_left, sl::VIEW::LEFT);
      zeds_[0].retrieveImage(image_right, sl::VIEW::RIGHT);
      zeds_[0].retrieveMeasure(depth_map, sl::MEASURE::DEPTH);

      depthImageToROSmsg(depth_map, "map", this->get_clock()->now(), zed_1_depth_);
      imageToROSmsg(image_right, "map", this->get_clock()->now(), zed_1_image_right_);
      imageToROSmsg(image_left, "map", this->get_clock()->now(), zed_1_image_left_);
    }
  }

  {
    sl::Mat image_right;
    sl::Mat image_left;
    sl::Mat depth_map;

    if (zeds_[1].grab() == sl::ERROR_CODE::SUCCESS) {
      zeds_[1].retrieveImage(image_left, sl::VIEW::LEFT);
      zeds_[1].retrieveImage(image_right, sl::VIEW::RIGHT);
      zeds_[1].retrieveMeasure(depth_map, sl::MEASURE::DEPTH);

      depthImageToROSmsg(depth_map, "map", this->get_clock()->now(), zed_2_depth_);
      imageToROSmsg(image_right, "map", this->get_clock()->now(), zed_2_image_right_);
      imageToROSmsg(image_left, "map", this->get_clock()->now(), zed_2_image_left_);

    }
  }

}

RCLCPP_COMPONENTS_REGISTER_NODE(ZedStreamer)

#endif // ZED_STREAMER_NODE__