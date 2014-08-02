//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "../include/quadrotor_interface.h"
#include "../include/pid.h"

#include <controller_interface/controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <limits>

namespace hector_quadrotor_controller {

using namespace controller_interface;

class TwistController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  TwistController()
    : node_handle_(0)
  {}

  ~TwistController()
  {
    is_shutting_down_ = true;
    callback_queue_thread_.join();

    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
      node_handle_ = 0;
    }
  }

  bool init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    // get interface handles
    pose_          = interface->getPose();
    twist_         = interface->getTwist();
    acceleration_  = interface->getAcceleration();
    twist_input_   = interface->addInput<TwistCommandHandle>("twist");
    wrench_output_ = interface->addOutput<WrenchCommandHandle>("wrench");
    interface->claim(wrench_output_->getName());

    // initialize NodeHandle
    delete node_handle_;
    node_handle_ = new ros::NodeHandle(root_nh);

    // subscribe to commanded twist (geometry_msgs/TwistStamped) and cmd_vel (geometry_msgs/Twist)
    ros::SubscribeOptions twist_subscribe_options = ros::SubscribeOptions::create<geometry_msgs::TwistStamped>(
      "command/twist", 1,
      boost::bind(&TwistController::twistCommandCallback, this, _1),
      ros::VoidConstPtr(), &callback_queue_
    );
    twist_subscriber_ = node_handle_->subscribe(twist_subscribe_options);

    ros::SubscribeOptions cmd_vel_subscribe_options = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      "cmd_vel", 1,
      boost::bind(&TwistController::cmd_velCommandCallback, this, _1),
      ros::VoidConstPtr(), &callback_queue_
    );
    cmd_vel_subscriber_ = node_handle_->subscribe(cmd_vel_subscribe_options);

    // engage/shutdown service servers
    {
      ros::AdvertiseServiceOptions ops = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        "engage", boost::bind(&TwistController::engageCallback, this, _1, _2),
        ros::VoidConstPtr(), &callback_queue_
      );
      engage_service_server_ = node_handle_->advertiseService(ops);

      ops = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        "shutdown", boost::bind(&TwistController::shutdownCallback, this, _1, _2),
        ros::VoidConstPtr(), &callback_queue_
      );
      shutdown_service_server_ = node_handle_->advertiseService(ops);
    }

    // initialize PID controllers
    pid_.linear.x.init(ros::NodeHandle(controller_nh, "linear/xy"));
    pid_.linear.y.init(ros::NodeHandle(controller_nh, "linear/xy"));
    pid_.linear.z.init(ros::NodeHandle(controller_nh, "linear/z"));
    pid_.angular.x.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.y.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.z.init(ros::NodeHandle(controller_nh, "angular/z"));

    // load other parameters
    controller_nh.getParam("auto_engage", auto_engage_ = true);
    controller_nh.getParam("limits/load_factor", load_factor_limit = 1.5);
    controller_nh.getParam("limits/force/z", limits_.force.z);
    controller_nh.getParam("limits/torque/xy", limits_.torque.x);
    controller_nh.getParam("limits/torque/xy", limits_.torque.y);
    controller_nh.getParam("limits/torque/z", limits_.torque.z);

    // get mass and inertia from QuadrotorInterface
    interface->getMassAndInertia(mass_, inertia_);

    command_given_in_stabilized_frame_ = false;

    is_shutting_down_ = false;
    callback_queue_thread_ = boost::thread(boost::bind(&TwistController::CallbackQueueThread, this));
    return true;
  }

  void reset()
  {
    pid_.linear.x.reset();
    pid_.linear.y.reset();
    pid_.linear.x.reset();
    pid_.angular.x.reset();
    pid_.angular.y.reset();
    pid_.angular.z.reset();

    wrench_.wrench.force.x  = 0.0;
    wrench_.wrench.force.y  = 0.0;
    wrench_.wrench.force.z  = 0.0;
    wrench_.wrench.torque.x = 0.0;
    wrench_.wrench.torque.y = 0.0;
    wrench_.wrench.torque.z = 0.0;

    linear_z_control_error_ = 0.0;
    motors_running_ = false;
  }

  void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    command_ = *command;
    if (command_.header.stamp.isZero()) command_.header.stamp = ros::Time::now();
    command_given_in_stabilized_frame_ = false;

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);
  }

  void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr& command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    command_.twist = *command;
    command_.header.stamp = ros::Time::now();
    command_given_in_stabilized_frame_ = true;

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);
  }

  bool engageCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("twist_controller", "Engaging motors!");
    motors_running_ = true;
  }

  bool shutdownCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
    motors_running_ = false;
  }

  void starting(const ros::Time &time)
  {
    reset();
    wrench_output_->start();
  }

  void stopping(const ros::Time &time)
  {
    wrench_output_->stop();
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // Get twist command input
    if (twist_input_->connected() && twist_input_->enabled()) {
      command_.twist = twist_input_->getCommand();
      command_given_in_stabilized_frame_ = false;
    }

    // Get current state and command
    Twist command = command_.twist;
    Twist twist = twist_->twist();
    Twist twist_body;
    twist_body.linear =  pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    // Transform to world coordinates if necessary (yaw only)
    if (command_given_in_stabilized_frame_) {
      double yaw = pose_->getYaw();
      Twist transformed = command;
      transformed.linear.x  = cos(yaw) * command.linear.x  - sin(yaw) * command.linear.y;
      transformed.linear.y  = sin(yaw) * command.linear.x  + cos(yaw) * command.linear.y;
      transformed.angular.x = cos(yaw) * command.angular.x - sin(yaw) * command.angular.y;
      transformed.angular.y = sin(yaw) * command.angular.x + cos(yaw) * command.angular.y;
      command = transformed;
    }

    // Auto engage/shutdown
    if (auto_engage_) {
      if (!motors_running_ && command.linear.z > 0.1) {
        motors_running_ = true;
        ROS_INFO_NAMED("twist_controller", "Engaging motors!");
      } else if (motors_running_ && command.linear.z < -0.1 && (twist.linear.z > -0.1 && twist.linear.z < 0.1)) {
        if (linear_z_control_error_ > 0.0) linear_z_control_error_ = 0.0; // positive control errors should not affect shutdown
        if (pid_.linear.z.getFilteredControlError(linear_z_control_error_, 10.0, period) < -1.0) {
          motors_running_ = false;
          ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
        }
      }
    }

    // Update output
    if (motors_running_) {
      // Get gravity and load factor
      const double gravity = 9.8065;
      double load_factor = 1. / (  pose_->pose().orientation.w * pose_->pose().orientation.w
                                 - pose_->pose().orientation.x * pose_->pose().orientation.x
                                 - pose_->pose().orientation.y * pose_->pose().orientation.y
                                 + pose_->pose().orientation.z * pose_->pose().orientation.z );
      // Note: load_factor could be NaN or Inf...?
      if (load_factor_limit > 0.0 && !(load_factor < load_factor_limit)) load_factor = load_factor_limit;

      Vector3 acceleration_command;
      acceleration_command.x = pid_.linear.x.update(command.linear.x, twist.linear.x, acceleration_->acceleration().x, period);
      acceleration_command.y = pid_.linear.y.update(command.linear.y, twist.linear.y, acceleration_->acceleration().y, period);
      acceleration_command.z = pid_.linear.z.update(command.linear.z, twist.linear.z, acceleration_->acceleration().z, period) + gravity;
      Vector3 acceleration_command_body = pose_->toBody(acceleration_command);

      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist.linear:               [" << twist.linear.x << " " << twist.linear.y << " " << twist.linear.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_body.angular:         [" << twist_body.angular.x << " " << twist_body.angular.y << " " << twist_body.angular.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.linear:       [" << command.linear.x << " " << command.linear.y << " " << command.linear.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.angular:      [" << command.angular.x << " " << command.angular.y << " " << command.angular.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration:               [" << acceleration_->acceleration().x << " " << acceleration_->acceleration().y << " " << acceleration_->acceleration().z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_world: [" << acceleration_command.x << " " << acceleration_command.y << " " << acceleration_command.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_body:  [" << acceleration_command_body.x << " " << acceleration_command_body.y << " " << acceleration_command_body.z << "]");

      wrench_.wrench.torque.x = inertia_[0] * pid_.angular.x.update(-acceleration_command_body.y / gravity, 0.0, twist_body.angular.x, period);
      wrench_.wrench.torque.y = inertia_[1] * pid_.angular.y.update( acceleration_command_body.x / gravity, 0.0, twist_body.angular.y, period);
      wrench_.wrench.torque.z = inertia_[2] * pid_.angular.z.update( command.angular.z, twist.angular.z, 0.0, period);
      wrench_.wrench.force.x  = 0.0;
      wrench_.wrench.force.y  = 0.0;
      wrench_.wrench.force.z  = mass_       * ((acceleration_command.z - gravity) * load_factor + gravity);

      if (limits_.force.z > 0.0 && wrench_.wrench.force.z > limits_.force.z) wrench_.wrench.force.z = limits_.force.z;
      if (wrench_.wrench.force.z <= 0.0) wrench_.wrench.force.z = std::numeric_limits<double>::min();
      if (limits_.torque.x > 0.0) {
        if (wrench_.wrench.torque.x >  limits_.torque.x) wrench_.wrench.torque.x =  limits_.torque.x;
        if (wrench_.wrench.torque.x < -limits_.torque.x) wrench_.wrench.torque.x = -limits_.torque.x;
      }
      if (limits_.torque.y > 0.0) {
        if (wrench_.wrench.torque.y >  limits_.torque.y) wrench_.wrench.torque.y =  limits_.torque.y;
        if (wrench_.wrench.torque.y < -limits_.torque.y) wrench_.wrench.torque.y = -limits_.torque.y;
      }
      if (limits_.torque.z > 0.0) {
        if (wrench_.wrench.torque.z >  limits_.torque.z) wrench_.wrench.torque.z =  limits_.torque.z;
        if (wrench_.wrench.torque.z < -limits_.torque.z) wrench_.wrench.torque.z = -limits_.torque.z;
      }

    } else {
      reset();
    }

    // set wrench output
    wrench_.header.stamp = time;
    wrench_.header.frame_id = "base_link";
    wrench_output_->setCommand(wrench_.wrench);
  }

private:
  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr acceleration_;
  TwistCommandHandlePtr twist_input_;
  WrenchCommandHandlePtr wrench_output_;

  ros::NodeHandle *node_handle_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::ServiceServer engage_service_server_;
  ros::ServiceServer shutdown_service_server_;

  geometry_msgs::TwistStamped command_;
  geometry_msgs::WrenchStamped wrench_;
  bool command_given_in_stabilized_frame_;

  struct {
    struct {
      PID x;
      PID y;
      PID z;
    } linear, angular;
  } pid_;

  geometry_msgs::Wrench limits_;
  bool auto_engage_;
  double load_factor_limit;
  double mass_;
  double inertia_[3];

  bool motors_running_;
  double linear_z_control_error_;

  bool is_shutting_down_;
  ros::CallbackQueue callback_queue_;
  boost::thread callback_queue_thread_;
  boost::mutex command_mutex_;

  void CallbackQueueThread() {
    static const ros::WallDuration timeout(1.0);
    while(!is_shutting_down_) {
      callback_queue_.callAvailable(timeout);
    }
  }
};

} // namespace hector_quadrotor_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller::TwistController, controller_interface::ControllerBase)
