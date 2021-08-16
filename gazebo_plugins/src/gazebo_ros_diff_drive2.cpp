// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * \file  gazebo_ros_diff_drive2.cpp
 *
 * \brief A differential drive plugin for gazebo with controller feedback.
 * Based on the original diffdrive plugin in this repository.
 *
 * \author Jonatan Olofsson (jonatan.olofsson@saabgroup.com)
 * \date 16th of August 2021
 *
 */

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
struct RollingMean {
    std::vector<double> values;
    size_t next_index = 0;
    double last_value = 0;

    RollingMean(const unsigned N = 50)
    : values(N, 0.0)
    {}

    void update(const double value) {
      values[next_index] = value;
      next_index = (next_index + 1) % values.size();
    }

    void update(const double ivalue, const double dt) {
      update((ivalue - last_value) / dt);
      last_value = ivalue;
    }

    double get() {
      return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    }
};

class GazeboRosDiffDrivePrivate
{
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
    /// Use an ancoder
    ENCODER = 0,

    /// Use ground truth from simulation world
    WORLD = 1,
  };

  /// Indicates which wheel
  enum
  {
    /// Right wheel
    RIGHT = 0,

    /// Left wheel
    LEFT = 1,
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelVelocities();

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection;

  /// Distance between the wheels, in meters.
  std::vector<double> wheel_separation;

  /// Radius of wheels, in meters.
  std::vector<double> wheel_radius;

  /// Maximum wheel torque, in Nm.
  double max_wheel_torque;

  /// Maximum wheel velocity
  double max_wheel_vel;

  /// Speed sent to wheel.
  std::vector<double> wheel_speed_instr;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints;

  /// Pointer to model.
  gazebo::physics::ModelPtr model;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;

  /// Protect variables accessed on callbacks.
  std::mutex lock;

  /// Linear velocity in X received on command (m/s).
  double target_v{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_w{0.0};

  /// Update period in seconds.
  double update_period;

  /// Last update time.
  gazebo::common::Time last_update_time;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder;

  /// Odometry frame ID
  std::string odometry_frame;

  /// Last time the encoder was updated
  gazebo::common::Time last_encoder_update;

  /// Either ENCODER or WORLD
  OdomSource odom_source;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom;

  /// Robot base frame ID
  std::string robot_base_frame;

  /// True to publish odometry messages.
  bool publish_odom;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf;

  /// Store number of wheel pairs
  unsigned int num_wheel_pairs;

  /// Covariance in odometry
  double covariance[6];

  /// Clamped commands
  double target_v_clamped, target_w_clamped;

  /// Current velocities
  double cur_v, cur_w;

  double v_cmd = 0;
  double w_cmd = 0;

  /// Rolling average velocities
  std::vector<RollingMean> avg_v;

  /// Linear Velocity controller
  gazebo::common::PID vpid;

  /// Angular Velocity controller
  gazebo::common::PID wpid;
};

GazeboRosDiffDrive::GazeboRosDiffDrive()
: impl_(std::make_unique<GazeboRosDiffDrivePrivate>())
{}

GazeboRosDiffDrive::~GazeboRosDiffDrive() {}

void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model = _model;

  // Initialize ROS node
  impl_->ros_node = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node->get_qos();

  // Get number of wheel pairs in the model
  impl_->num_wheel_pairs = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs", 1).first);

  if (impl_->num_wheel_pairs < 1) {
    impl_->num_wheel_pairs = 1;
    RCLCPP_WARN(
      impl_->ros_node->get_logger(),
      "Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
  }

  // Dynamic properties
  impl_->max_wheel_torque = _sdf->Get<double>("max_wheel_torque", 5.0).first;
  impl_->max_wheel_vel = _sdf->Get<double>("max_wheel_velocity", 1.0).first;

  // Get joints and Kinematic properties
  std::vector<gazebo::physics::JointPtr> left_joints, right_joints;

  for (auto left_joint_elem = _sdf->GetElement("left_joint"); left_joint_elem != nullptr;
    left_joint_elem = left_joint_elem->GetNextElement("left_joint"))
  {
    auto left_joint_name = left_joint_elem->Get<std::string>();
    auto left_joint = _model->GetJoint(left_joint_name);
    if (!left_joint) {
      RCLCPP_ERROR(
        impl_->ros_node->get_logger(),
        "Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
      impl_->ros_node.reset();
      return;
    }
    left_joint->SetParam("fmax", 0, impl_->max_wheel_torque);
    left_joints.push_back(left_joint);
  }

  for (auto right_joint_elem = _sdf->GetElement("right_joint"); right_joint_elem != nullptr;
    right_joint_elem = right_joint_elem->GetNextElement("right_joint"))
  {
    auto right_joint_name = right_joint_elem->Get<std::string>();
    auto right_joint = _model->GetJoint(right_joint_name);
    if (!right_joint) {
      RCLCPP_ERROR(
        impl_->ros_node->get_logger(),
        "Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
      impl_->ros_node.reset();
      return;
    }
    right_joint->SetParam("fmax", 0, impl_->max_wheel_torque);
    right_joints.push_back(right_joint);
  }

  if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs) {
    RCLCPP_ERROR(
      impl_->ros_node->get_logger(),
      "Inconsistent number of joints specified. Plugin will not work.");
    impl_->ros_node.reset();
    return;
  }

  auto pid = _sdf->Get<ignition::math::Vector3d>(
    "v_pid_gains", ignition::math::Vector3d::Zero).first;
  auto i_range = _sdf->Get<ignition::math::Vector2d>(
    "v_i_range", ignition::math::Vector2d::Zero).first;
  impl_->vpid.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());
  pid = _sdf->Get<ignition::math::Vector3d>(
    "w_pid_gains", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "w_i_range", ignition::math::Vector2d::Zero).first;
  impl_->wpid.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  unsigned int index;
  for (index = 0; index < impl_->num_wheel_pairs; ++index) {
    impl_->joints.push_back(right_joints[index]);
    impl_->joints.push_back(left_joints[index]);
  }
  impl_->wheel_separation.assign(impl_->num_wheel_pairs, 0.34);
  for (auto wheel_separation = _sdf->GetElement("wheel_separation"); wheel_separation != nullptr;
    wheel_separation = wheel_separation->GetNextElement("wheel_separation"))
  {
    if (index >= impl_->num_wheel_pairs) {
      RCLCPP_WARN(impl_->ros_node->get_logger(), "Ignoring rest of specified <wheel_separation>");
      break;
    }
    impl_->wheel_separation[index] = wheel_separation->Get<double>();
    RCLCPP_INFO(
      impl_->ros_node->get_logger(),
      "Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation[index]);
    index++;
  }

  index = 0;
  impl_->wheel_radius.assign(impl_->num_wheel_pairs, 0.075);
  for (auto wheel_diameter = _sdf->GetElement("wheel_radius"); wheel_diameter != nullptr;
    wheel_diameter = wheel_diameter->GetNextElement("wheel_radius"))
  {
    if (index >= impl_->num_wheel_pairs) {
      RCLCPP_WARN(impl_->ros_node->get_logger(), "Ignoring rest of specified <wheel_radius>");
      break;
    }
    impl_->wheel_radius[index] = wheel_diameter->Get<double>() * 0.5;
    RCLCPP_INFO(
      impl_->ros_node->get_logger(),
      "Wheel pair %i radius set to [%fm]", index + 1, impl_->wheel_radius[index]);
    index++;
  }

  impl_->wheel_speed_instr.assign(2 * impl_->num_wheel_pairs, 0);

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period = 1.0 / update_rate;
  } else {
    impl_->update_period = 0.0;
  }
  impl_->last_update_time = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub = impl_->ros_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub->get_topic_name());

  // Odometry
  impl_->odometry_frame = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source = static_cast<GazeboRosDiffDrivePrivate::OdomSource>(
    _sdf->Get<int>("odometry_source", 1).first);

  int avg_size = _sdf->Get<int>("avg_size ", 100).first;
  if (impl_->odom_source == impl_->ENCODER) {
    impl_->avg_v.resize(2 * impl_->num_wheel_pairs);
    avg_size *= 2;
  } else if (impl_->odom_source == impl_->WORLD) {
    impl_->avg_v.resize(3); // w, vx, vy
  }
  for (unsigned i = 0; i < impl_->avg_v.size(); ++i) {
    impl_->avg_v[i].values.resize(avg_size);
  }

  // Advertise odometry topic
  impl_->publish_odom = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom) {
    impl_->odometry_pub = impl_->ros_node->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf || impl_->publish_odom_tf) {
    impl_->transform_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node);

    if (impl_->publish_odom_tf) {
      RCLCPP_INFO(
        impl_->ros_node->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame.c_str(),
        impl_->robot_base_frame.c_str());
    }

    for (index = 0; index < impl_->num_wheel_pairs; ++index) {
      if (impl_->publish_wheel_tf) {
        RCLCPP_INFO(
          impl_->ros_node->get_logger(),
          "Publishing wheel transforms between [%s], [%s] and [%s]",
          impl_->robot_base_frame.c_str(),
          impl_->joints[2 * index + GazeboRosDiffDrivePrivate::LEFT]->GetName().c_str(),
          impl_->joints[2 * index + GazeboRosDiffDrivePrivate::RIGHT]->GetName().c_str());
      }
    }
  }

  impl_->covariance[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;
  impl_->covariance[3] = _sdf->Get<double>("covariance_vx", 0.00001).first;
  impl_->covariance[4] = _sdf->Get<double>("covariance_vy", 0.00001).first;
  impl_->covariance[5] = _sdf->Get<double>("covariance_vyaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosDiffDrive::Reset()
{
  impl_->last_update_time =
    impl_->joints[GazeboRosDiffDrivePrivate::LEFT]->GetWorld()->SimTime();
  for (unsigned int i = 0; i < impl_->num_wheel_pairs; ++i) {
    if (impl_->joints[2 * i + GazeboRosDiffDrivePrivate::LEFT] &&
      impl_->joints[2 * i + GazeboRosDiffDrivePrivate::RIGHT])
    {
      impl_->joints[2 * i + GazeboRosDiffDrivePrivate::LEFT]->SetParam(
        "fmax", 0, impl_->max_wheel_torque);
      impl_->joints[2 * i + GazeboRosDiffDrivePrivate::RIGHT]->SetParam(
        "fmax", 0, impl_->max_wheel_torque);
    }
  }
  impl_->pose_encoder.x = 0;
  impl_->pose_encoder.y = 0;
  impl_->pose_encoder.theta = 0;
  impl_->target_v = 0;
  impl_->target_w = 0;
  impl_->vpid.Reset();
  impl_->wpid.Reset();
}

void GazeboRosDiffDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosDiffDrivePrivate::OnUpdate");
#endif
  // Update encoder even if we're going to skip this update
  if (odom_source == ENCODER) {
    UpdateOdometryEncoder(_info.simTime);
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom message if using ground truth
  if (odom_source == WORLD) {
    UpdateOdometryWorld();
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

  double seconds_since_last_update = (_info.simTime - last_update_time).Double();

  if (seconds_since_last_update >= update_period) {

#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
    if (publish_odom) {
      PublishOdometryMsg(_info.simTime);
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
    if (publish_wheel_tf) {
      PublishWheelsTf(_info.simTime);
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
    if (publish_odom_tf) {
      PublishOdometryTf(_info.simTime);
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("UpdateWheelVelocities");
#endif
  // Update robot in case new velocities have been requested
    UpdateWheelVelocities();
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    v_cmd = vpid.Update(cur_v - target_v_clamped, seconds_since_last_update);
    w_cmd = wpid.Update(cur_w - target_w_clamped, seconds_since_last_update);
    //v_cmd = target_v_clamped;
    //w_cmd = target_w_clamped;

    last_update_time = _info.simTime;
  }
  for (unsigned i = 0; i < num_wheel_pairs; ++i) {
    auto va = w_cmd * wheel_separation[i] * 0.5;
    joints[i * 2 + LEFT]->SetForce(0, (v_cmd - va) / wheel_radius[i]);
    joints[i * 2 + RIGHT]->SetForce(0, (v_cmd + va) / wheel_radius[i]);
    //joints[i * 2 + LEFT]->SetVelocity(0, (v_cmd - va) / wheel_radius[i]);
    //joints[i * 2 + RIGHT]->SetVelocity(0, (v_cmd + va) / wheel_radius[i]);
  }
  //RCLCPP_INFO(ros_node->get_logger(), "Update joints: \n%f : %f / %f : %f / %f : %f / %f : %f / %f : %f",
      //cur_v - target_v_clamped,
      //cur_w - target_w_clamped,
      //v_cmd,
      //w_cmd,
      //joints[0]->GetVelocity(0),
      //joints[1]->GetVelocity(0),
      //joints[0]->GetForce(0),
      //joints[1]->GetForce(0),
      //joints[2]->GetForce(0),
      //joints[3]->GetForce(0)
      //);

}

void GazeboRosDiffDrivePrivate::UpdateWheelVelocities()
{
  std::lock_guard<std::mutex> scoped_lock(lock);

  double vr = fmin(max_wheel_vel, fmax(target_v, -max_wheel_vel));
  double va = target_w;

  // Limit the angular velocity
  for (unsigned int i = 0; i < num_wheel_pairs; ++i) {
    auto rotvel = va * wheel_separation[i] / 2.0;
    va = fmin(max_wheel_vel, fmax(rotvel, -max_wheel_vel)) * 2.0 / wheel_separation[i];
  }

  // Limit the linear velocity
  for (unsigned int i = 0; i < num_wheel_pairs; ++i) {
    auto rotvel = fabs(va * wheel_separation[i] / 2.0);
         if (vr + rotvel > max_wheel_vel)  vr =  max_wheel_vel - rotvel;
    else if (vr - rotvel > max_wheel_vel)  vr =  max_wheel_vel + rotvel;
    else if (vr - rotvel < -max_wheel_vel) vr = -max_wheel_vel + rotvel;
    else if (vr + rotvel < -max_wheel_vel) vr = -max_wheel_vel - rotvel;
  }

  target_v_clamped = vr;
  target_w_clamped = va;
}

void GazeboRosDiffDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock);
  target_v = _msg->linear.x;
  target_w = _msg->angular.z;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
{
  double seconds_since_last_update = (_current_time - last_encoder_update).Double();
  last_encoder_update = _current_time;

  //std::cout << "Joint position: \n";
  for (unsigned i = 0; i < joints.size(); ++i) {
    avg_v[i].update(joints[i]->Position(0), seconds_since_last_update);
  }
  double ssum = 0;
  double sdiff = 0;
  double dtheta = 0;
  for (unsigned i = 0; i < num_wheel_pairs; ++i) {
    double vl = avg_v[2 * i + LEFT].get();
    double vr = avg_v[2 * i + RIGHT].get();

    double b = wheel_separation[i];

    // Book: Sigwart 2011 Autonomous Mobile Robots page:337
    double sl = vl * wheel_radius[i] * seconds_since_last_update;
    double sr = vr * wheel_radius[i] * seconds_since_last_update;
    ssum += sl + sr;
    sdiff += sr - sl;
    dtheta += (sr - sl) / b;
  }
  ssum /= num_wheel_pairs;
  sdiff /= num_wheel_pairs;
  dtheta /= num_wheel_pairs;
  double v = ssum / seconds_since_last_update;
  double w = dtheta / seconds_since_last_update;

  //std::cout << "v: " << v << ", w: " << w << ", dt: " << seconds_since_last_update << std::endl;
  //if (fabs(v) > 100) for (;;);

  double dx = (ssum) / 2.0 * cos(pose_encoder.theta + dtheta / 2);
  double dy = (ssum) / 2.0 * sin(pose_encoder.theta + dtheta / 2);

  pose_encoder.x += dx;
  pose_encoder.y += dy;
  pose_encoder.theta += dtheta;

  cur_w = odom.twist.twist.angular.z = w;
  cur_v = odom.twist.twist.linear.x = v;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder.theta);
  vt = tf2::Vector3(pose_encoder.x, pose_encoder.y, 0);

  odom.pose.pose.position.x = vt.x();
  odom.pose.pose.position.y = vt.y();
  odom.pose.pose.position.z = vt.z();

  odom.pose.pose.orientation.x = qt.x();
  odom.pose.pose.orientation.y = qt.y();
  odom.pose.pose.orientation.z = qt.z();
  odom.pose.pose.orientation.w = qt.w();

  odom.twist.twist.linear.y = 0;
  //std::cout << std::endl;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryWorld()
{
  auto pose = model->WorldPose();
  odom.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model->WorldLinearVel();

  // Convert velocity to child_frame_id(aka base_footprint)
  float yaw = pose.Rot().Yaw();
  auto linear_x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  auto linear_y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  avg_v[0].update(model->WorldAngularVel().Z());
  avg_v[1].update(linear_x);
  avg_v[2].update(linear_y);

  cur_w = avg_v[0].get();
  cur_v = avg_v[1].get();

  odom.twist.twist.angular.z = cur_w;
  odom.twist.twist.linear.x = cur_v;
  odom.twist.twist.linear.y = avg_v[2].get();
}

void GazeboRosDiffDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame;
  msg.child_frame_id = robot_base_frame;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom.pose.pose.position);
  msg.transform.rotation = odom.pose.pose.orientation;

  transform_broadcaster->sendTransform(msg);
}

void GazeboRosDiffDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (unsigned int i = 0; i < 2 * num_wheel_pairs; ++i) {
    auto pose_wheel = joints[i]->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints[i]->GetParent()->GetName();
    msg.child_frame_id = joints[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster->sendTransform(msg);
  }
}

void GazeboRosDiffDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom.pose.covariance[0] = covariance[0];
  odom.pose.covariance[7] = covariance[1];
  odom.pose.covariance[14] = 1000000000000.0;
  odom.pose.covariance[21] = 1000000000000.0;
  odom.pose.covariance[28] = 1000000000000.0;
  odom.pose.covariance[35] = covariance[2];

  odom.twist.covariance[0] = covariance[3];
  odom.twist.covariance[7] = covariance[4];
  odom.twist.covariance[14] = 1000000000000.0;
  odom.twist.covariance[21] = 1000000000000.0;
  odom.twist.covariance[28] = 1000000000000.0;
  odom.twist.covariance[35] = covariance[5];

  // Set header
  odom.header.frame_id = odometry_frame;
  odom.child_frame_id = robot_base_frame;
  odom.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub->publish(odom);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
}  // namespace gazebo_plugins
