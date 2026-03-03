#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/string.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2/convert.hpp>
#include <string>
#include <cmath>
#include "boost/thread.hpp"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "utils.h"
#include <mutex>
#include <map>
#include <iostream>
#include <chrono>


std::mutex tf_mutex;

using json = nlohmann::json;

//Custom struct to store observed detections
struct Detection {
    char robot;
    int marker_id;
    geometry_msgs::msg::TransformStamped tf_detection;
    int n_observations;
};

struct ObjectDetection {
    char robot;
    std::string object_class;
    geometry_msgs::msg::TransformStamped tf_object;
    int n_observations;
};

class DetectionPublisher : public rclcpp::Node
{
    public:
        DetectionPublisher();

        void detection_cb( const geometry_msgs::msg::TransformStamped::SharedPtr msg );
        void detection_rover_cb( const geometry_msgs::msg::TransformStamped::SharedPtr msg );
        void object_detection_cb( const std_msgs::msg::String::SharedPtr msg );
        // void object_detection_cb( const geometry_msgs::msg::PoseArray::SharedPtr msg );
        void parse_json_data(const json& json_data);
        std::string extract_json_from_string(const std::string& input);
        void listener_tf();
        void listener_rover_tf();
        void run();
        void publish_tf();
        void object_tf_pub();

    private:

        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr _subscription;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr _subscription_rover;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_object;

        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_rover_broadcaster;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_object_broadcaster;
        rclcpp::TimerBase::SharedPtr _timer_tf_out;
        rclcpp::TimerBase::SharedPtr _timer_tf_in;
        rclcpp::TimerBase::SharedPtr _timer_rover_tf_in;
        rclcpp::TimerBase::SharedPtr _timer_object_tf;

        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};

        std::unique_ptr<tf2_ros::Buffer> _tf_buffer_rover;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener_rover{nullptr};

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;

        std::vector<Detection> _detections_vector;
        std::vector<Detection> _rover_detections_vector;
        std::vector<ObjectDetection> _objects_vector;

        Eigen::Vector3d _p_cam_to_map = Eigen::Vector3d::Zero();
        Eigen::Matrix3d _R_cam_to_map = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d _R_cam_to_aruco = Eigen::Matrix3d::Identity();
        Eigen::Vector4d _q_cam_to_map;
        Eigen::Vector4d _q_cam_to_aruco;
        Eigen::Vector4d _q_map_to_aruco;
        Eigen::Vector4d _p_temp_c = Eigen::Vector4d::Zero();
        Eigen::Vector4d _p_temp_c2m = Eigen::Vector4d::Zero();
        Eigen::Matrix4d _T_cam_to_map = Eigen::Matrix4d::Zero();
        Eigen::Matrix4d _T_cam_to_aruco = Eigen::Matrix4d::Zero();
        Eigen::Matrix4d _T_map_to_aruco = Eigen::Matrix4d::Zero();

        Eigen::Vector3d _p_cam_to_map_rover = Eigen::Vector3d::Zero();
        Eigen::Matrix3d _R_cam_to_map_rover = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d _R_cam_to_aruco_rover = Eigen::Matrix3d::Identity();
        Eigen::Vector4d _q_cam_to_map_rover;
        Eigen::Vector4d _q_cam_to_aruco_rover;
        Eigen::Vector4d _q_map_to_aruco_rover;
        Eigen::Vector4d _p_temp_c_rover = Eigen::Vector4d::Zero();
        Eigen::Vector4d _p_temp_c2m_rover = Eigen::Vector4d::Zero();
        Eigen::Matrix4d _T_cam_to_map_rover = Eigen::Matrix4d::Zero();
        Eigen::Matrix4d _T_cam_to_aruco_rover = Eigen::Matrix4d::Zero();
        Eigen::Matrix4d _T_map_to_aruco_rover = Eigen::Matrix4d::Zero();

        Eigen::Matrix3d _R_cam_to_front;

        geometry_msgs::msg::TransformStamped _t_rover;

        tf2::Matrix3x3 _ocv_to_pos;
        tf2::Transform _ocv_to_pos_tf;
        bool _already_present=false;
        bool _already_present_rover=false;
        int _detection_pos = 0;
        bool _added = false;
        bool _existing_id=false;
        bool _object_already_present=false;
        char _robot_id = 'D';

};

