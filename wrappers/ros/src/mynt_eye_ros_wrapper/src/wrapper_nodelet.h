#ifndef WRAPPER_NODELET_H_
#define WRAPPER_NODELET_H_



#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Dense>

#include <mynt_eye_ros_msgs/srv/get_info.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <string>
#include <cassert>
 
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"


MYNTEYE_BEGIN_NAMESPACE

class ROSWrapperNodelet {
public:
    ROSWrapperNodelet(rclcpp::Node::SharedPtr &node);
    ~ROSWrapperNodelet();

    rclcpp::Time hardTimeToSoftTime(std::uint64_t _hard_time);
    inline bool is_overflow(std::uint64_t now,
      std::uint64_t pre) {

        return (now < pre) && ((pre - now) > (unit_hard_time / 2));
    }

    inline bool is_repeated(std::uint64_t now,
        std::uint64_t pre) {
        return now == pre;
    }

    inline bool is_abnormal(std::uint32_t now,
        std::uint32_t pre) {

        return (now < pre) && ((pre - now) < (unit_hard_time / 4));
    }

    rclcpp::Time checkUpTimeStamp(std::uint64_t _hard_time,
        const Stream &stream);
    rclcpp::Time checkUpImuTimeStamp(std::uint64_t _hard_time);

    void onInit();
    bool getInfo(
      const mynt_eye_ros_msgs::srv::GetInfo::Request::SharedPtr req,     // NOLINT
      mynt_eye_ros_msgs::srv::GetInfo::Response::SharedPtr res);
    void publishTopics();
    void publishData(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      rclcpp::Time stamp);
    int getStreamSubscribers(const Stream &stream);
    void publishOthers(const Stream &stream);
    void publishCamera(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      rclcpp::Time stamp);
    void publishMono(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      rclcpp::Time stamp);

    void publishPoints(
      const api::StreamData &data, std::uint32_t seq, rclcpp::Time stamp);
    void publishImu(
      const api::MotionData &data, std::uint32_t seq, rclcpp::Time stamp);
    void timestampAlign();
    void publishImuBySync();
    void publishTemperature(
        float temperature, std::uint32_t seq, rclcpp::Time stamp);

private:
    void initDevice();
    std::shared_ptr<Device> selectDevice();
    std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics();
    std::shared_ptr<Extrinsics> getDefaultExtrinsics();
    void publishMesh();
    void computeRectTransforms();
    sensor_msgs::msg::CameraInfo::SharedPtr  getCameraInfo(const Stream &stream);
    void publishStaticTransforms();
private:
  // ros::NodeHandle nh_;
  // ros::NodeHandle private_nh_;

  pthread_mutex_t mutex_data_;

  Model model_;
  std::map<Option, std::string> option_names_;
  // camera:
  //   LEFT, RIGHT, LEFT_RECTIFIED, RIGHT_RECTIFIED,
  //   DISPARITY, DISPARITY_NORMALIZED,
  //   DEPTH
  std::map<Stream, image_transport::CameraPublisher> camera_publishers_;
  std::map<Stream, sensor_msgs::msg::CameraInfo::SharedPtr> camera_info_ptrs_;
  std::map<Stream, std::string> camera_encodings_;

  // image: LEFT_RECTIFIED, RIGHT_RECTIFIED, DISPARITY, DISPARITY_NORMALIZED,
  // DEPTH
  std::map<Stream, image_transport::Publisher> image_publishers_;
  std::map<Stream, std::string> image_encodings_;

  // mono: LEFT, RIGHT
  std::map<Stream, image_transport::Publisher> mono_publishers_;

  // pointcloud: POINTS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mesh_;  // < The publisher for camera mesh.
  visualization_msgs::msg::Marker mesh_msg_;  // < Mesh message.

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // ros::ServiceServer get_info_service_;
  ::rclcpp::Service<mynt_eye_ros_msgs::srv::GetInfo>::SharedPtr get_info_service_;

  // node params

  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string temperature_frame_id_;
  std::map<Stream, std::string> frame_ids_;

  double gravity_;

  // disparity type
  DisparityComputingMethod disparity_type_;
  // api

  std::shared_ptr<API> api_;

  // rectification transforms
  cv::Mat left_r_, right_r_, left_p_, right_p_, q_;
  cv::Rect left_roi_, right_roi_;

  double time_beg_ = -1;
  double left_time_beg_ = -1;
  double right_time_beg_ = -1;
  double imu_time_beg_ = -1;
  std::size_t left_count_ = 0;
  std::size_t right_count_ = 0;
  std::size_t imu_count_ = 0;
  std::size_t imu_sync_count_ = 0;
  std::shared_ptr<ImuData> imu_accel_;
  std::shared_ptr<ImuData> imu_gyro_;
  bool publish_imu_by_sync_ = true;
  std::map<Stream, bool> is_published_;
  bool is_motion_published_;
  bool is_started_;
  int frame_rate_;
  bool is_intrinsics_enable_;
  std::vector<ImuData> imu_align_;
  int skip_tag;
  int skip_tmp_left_tag;
  int skip_tmp_right_tag;
  double mesh_rotation_x;
  double mesh_rotation_y;
  double mesh_rotation_z;
  double mesh_position_x;
  double mesh_position_y;
  double mesh_position_z;
  std::vector<int64_t> left_timestamps;
  std::vector<int64_t> right_timestamps;

  std::uint64_t unit_hard_time = std::numeric_limits<std::uint32_t>::max();

  rclcpp::Node::SharedPtr node_;

  ::rclcpp::TimerBase::SharedPtr publish_timer_;
};


MYNTEYE_END_NAMESPACE
#endif