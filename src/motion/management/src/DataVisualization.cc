#include "DataVisualization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

using namespace rclcpp;

namespace clear {

DataVisualization::DataVisualization(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);

  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string robot_name = config_["model"]["name"].as<std::string>();

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("DataVisualization"), "model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();

  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();

  base_name = config_["model"]["base_name"].as<std::string>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_system_default);
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*nodeHandle_);
  joint_state_publisher_ =
      nodeHandle_->create_publisher<sensor_msgs::msg::JointState>(
          "joint_states", qos);
  foot_traj_msg_publisher_ =
      nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(
          topic_prefix + "foot_traj_vis", qos);
  foot_traj_ref_msg_publisher_ =
      nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(
          topic_prefix + "foot_traj_ref_vis", qos);
  footholds_pub_ =
      nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(
          topic_prefix + "footholds_vis", qos);
  base_traj_pub_ =
      nodeHandle_->create_publisher<visualization_msgs::msg::Marker>(
          topic_prefix + "base_trajectory", qos);
  base_traj_ref_pub_ =
      nodeHandle_->create_publisher<visualization_msgs::msg::Marker>(
          topic_prefix + "base_trajectory_ref", qos);
  line_strip_base.header.frame_id = "world";
  line_strip_base.action = visualization_msgs::msg::Marker::ADD;
  line_strip_base.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip_base.scale.x = 0.01;
  line_strip_base.scale.y = 0.01;
  line_strip_base.scale.z = 0.01;
  line_strip_base.color.a = 0.6;
  line_strip_base.color.r = 0.0;
  line_strip_base.color.g = 0.0;
  line_strip_base.color.b = 0.0;
  line_strip_base.ns = base_name;

  line_strip_base_ref.header.frame_id = "world";
  line_strip_base_ref.action = visualization_msgs::msg::Marker::ADD;
  line_strip_base_ref.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip_base_ref.scale.x = 0.01;
  line_strip_base_ref.scale.y = 0.01;
  line_strip_base_ref.scale.z = 0.01;
  line_strip_base_ref.color.a = 0.6;
  line_strip_base_ref.color.r = 1.0;
  line_strip_base_ref.color.g = 0.0;
  line_strip_base_ref.color.b = 0.0;
  line_strip_base_ref.ns = base_name;

  line_strip_foot_traj_.markers.resize(foot_names.size());
  line_strip_foot_traj_ref_.markers.resize(foot_names.size());
  line_strip_footholds_.markers.resize(foot_names.size());

  for (size_t i = 0; i < foot_names.size(); i++) {
    line_strip_foot_traj_.markers[i].header.frame_id = "world";
    line_strip_foot_traj_.markers[i].action =
        visualization_msgs::msg::Marker::ADD;
    line_strip_foot_traj_.markers[i].type =
        visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_foot_traj_.markers[i].color.a = 0.6;
    line_strip_foot_traj_.markers[i].color.r = 0.0;
    line_strip_foot_traj_.markers[i].color.g = 0.0;
    line_strip_foot_traj_.markers[i].color.b = 0.0;
    line_strip_foot_traj_.markers[i].scale.x = 0.01;
    line_strip_foot_traj_.markers[i].scale.y = 0.01;
    line_strip_foot_traj_.markers[i].scale.z = 0.01;
    line_strip_foot_traj_.markers[i].ns = foot_names[i];

    line_strip_foot_traj_ref_.markers[i].header.frame_id = "world";
    line_strip_foot_traj_ref_.markers[i].action =
        visualization_msgs::msg::Marker::ADD;
    line_strip_foot_traj_ref_.markers[i].type =
        visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_foot_traj_ref_.markers[i].color.a = 0.6;
    line_strip_foot_traj_ref_.markers[i].color.r = 1.0;
    line_strip_foot_traj_ref_.markers[i].color.g = 0.0;
    line_strip_foot_traj_ref_.markers[i].color.b = 0.0;
    line_strip_foot_traj_ref_.markers[i].scale.x = 0.01;
    line_strip_foot_traj_ref_.markers[i].scale.y = 0.01;
    line_strip_foot_traj_ref_.markers[i].scale.z = 0.01;
    line_strip_foot_traj_ref_.markers[i].ns = foot_names[i];

    line_strip_footholds_.markers[i].header.frame_id = "world";
    line_strip_footholds_.markers[i].action =
        visualization_msgs::msg::Marker::ADD;
    line_strip_footholds_.markers[i].type =
        visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip_footholds_.markers[i].color.a = 0.7;
    line_strip_footholds_.markers[i].color.r = 0.0;
    line_strip_footholds_.markers[i].color.g = 0.6;
    line_strip_footholds_.markers[i].color.b = 0.6;
    line_strip_footholds_.markers[i].scale.x = 0.04;
    line_strip_footholds_.markers[i].scale.y = 0.04;
    line_strip_footholds_.markers[i].scale.z = 0.04;
    line_strip_footholds_.markers[i].ns = foot_names[i];
  }

  inner_loop_thread_ = std::thread(&DataVisualization::innerLoop, this);
  run_.push(true);
}

DataVisualization::~DataVisualization() {
  run_.push(false);
  inner_loop_thread_.join();
}

void DataVisualization::innerLoop() {
  rclcpp::Rate loop_rate(500);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(10)); // very important, why?
  size_t iter = 0;
  while (rclcpp::ok() && run_.get()) {
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
    std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();

    pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

    /*--------- current state --------------*/
    publishCurrentState();

    if (referenceBuffer_ == nullptr || !referenceBuffer_->isReady()) {
      continue;
    }

    /*--------- foot trajectory --------------*/
    if (iter % 5 == 0) {
      publishFootTrajectory();
    }

    /*--------- Base Trajectory --------------*/
    if (iter % 10 == 0) {
      publishBaseTrajectory();
    }

    /*--------- footholds --------------*/
    if (iter % 50 == 0) {
      publishFootholds();
    }

    iter++;

    loop_rate.sleep();
  }
}

void DataVisualization::publishFootholds() {
  auto footholds = referenceBuffer_->getFootholds();
  if (footholds.empty()) {
    return;
  }
  if (footholds.size() == foot_names.size()) {
    for (size_t i = 0; i < foot_names.size(); i++) {
      auto &marker = line_strip_footholds_.markers[i];
      marker.header.stamp = nodeHandle_->now();
      marker.points.clear();
      geometry_msgs::msg::Point point;
      vector3_t pos = footholds[foot_names[i]].second;
      point.x = pos.x();
      point.y = pos.y();
      point.z = pos.z();
      marker.points.emplace_back(point);
    }
    footholds_pub_->publish(line_strip_footholds_);
  }
}

void DataVisualization::publishBaseTrajectory() {
  auto base_pos_traj_int = referenceBuffer_->getLipBasePosTraj();
  auto base_pos_traj = referenceBuffer_->getOptimizedBasePosTraj();
  if (base_pos_traj_int == nullptr || base_pos_traj == nullptr)
    return;

  line_strip_base.header.stamp = nodeHandle_->now();
  line_strip_base_ref.header.stamp = nodeHandle_->now();

  line_strip_base.points.clear();
  const scalar_t time_dur1 = base_pos_traj_int->duration();
  for (size_t i = 0; i * 0.02 < time_dur1; i++) {
    vector3_t pos =
        base_pos_traj_int->evaluate(i * 0.02 + base_pos_traj_int->ts());
    geometry_msgs::msg::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z();
    line_strip_base.points.emplace_back(point);
  }
  base_traj_pub_->publish(line_strip_base);

  line_strip_base_ref.points.clear();
  const scalar_t time_dur2 = base_pos_traj->duration();
  for (size_t i = 0; i * 0.02 < time_dur2; i++) {
    vector3_t pos = base_pos_traj->evaluate(i * 0.02 + base_pos_traj->ts());
    geometry_msgs::msg::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z();
    line_strip_base_ref.points.emplace_back(point);
  }
  base_traj_ref_pub_->publish(line_strip_base_ref);
}

void DataVisualization::publishFootTrajectory() {
  auto foot_traj = referenceBuffer_->getFootPosTraj();
  if (foot_traj.size() == foot_names.size()) {
    for (size_t i = 0; i < foot_names.size(); i++) {
      auto &marker = line_strip_foot_traj_.markers[i];
      auto &ref_marker = line_strip_foot_traj_ref_.markers[i];

      marker.header.stamp = nodeHandle_->now();
      ref_marker.header.stamp = nodeHandle_->now();

      if (marker.points.size() > 200) {
        marker.points.erase(marker.points.begin());
      }

      geometry_msgs::msg::Point point;
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_names[i]).translation();
      point.x = pos.x();
      point.y = pos.y();
      point.z = pos.z();
      marker.points.emplace_back(point);

      ref_marker.points.clear();
      auto foot_traj_i = foot_traj[foot_names[i]];
      const scalar_t time_dur = foot_traj_i->duration();
      for (size_t i = 0; i * 0.02 < time_dur; i++) {
        pos = foot_traj_i->evaluate(i * 0.02 + foot_traj_i->ts());
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        ref_marker.points.emplace_back(point);
      }
    }
    foot_traj_msg_publisher_->publish(line_strip_foot_traj_);
    foot_traj_ref_msg_publisher_->publish(line_strip_foot_traj_ref_);
  }
}

void DataVisualization::publishCurrentState() {
  geometry_msgs::msg::TransformStamped base_pose_msg;
  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  Eigen::Quaternion<scalar_t> quat = toQuaternion(base_pose.rotation());

  base_pose_msg.header.stamp = nodeHandle_->now();
  base_pose_msg.header.frame_id = "world";
  base_pose_msg.child_frame_id = base_name;
  base_pose_msg.transform.translation.x = base_pose.translation().x(),
  base_pose_msg.transform.translation.y = base_pose.translation().y(),
  base_pose_msg.transform.translation.z = base_pose.translation().z();
  base_pose_msg.transform.rotation.w = quat.w();
  base_pose_msg.transform.rotation.x = quat.x();
  base_pose_msg.transform.rotation.y = quat.y();
  base_pose_msg.transform.rotation.z = quat.z();

  sensor_msgs::msg::JointState jnt_state_msg;
  jnt_state_msg.header.frame_id = robot_name;
  jnt_state_msg.header.stamp = nodeHandle_->now();
  const auto &model_ = pinocchioInterface_ptr_->getModel();
  vector_t qpos = *qpos_ptr_buffer.get();
  for (const auto &joint_name : actuated_joints_name) {
    if (model_.existJointName(joint_name)) {
      pin::Index id = model_.getJointId(joint_name) - 2;
      jnt_state_msg.name.push_back(joint_name);
      jnt_state_msg.position.push_back(qpos[id + 7]);
    }
  }
  tf_broadcaster_->sendTransform(base_pose_msg);
  joint_state_publisher_->publish(jnt_state_msg);
}

void DataVisualization::updateCurrentState(std::shared_ptr<vector_t> qpos_ptr,
                                           std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void DataVisualization::updateReferenceBuffer(
    std::shared_ptr<ReferenceBuffer> referenceBuffer) {
  referenceBuffer_ = referenceBuffer;
}

} // namespace clear
