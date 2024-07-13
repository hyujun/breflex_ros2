#include "MuJoCoMessageHandler.h"

// #include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

namespace breflex
{

  MuJoCoMessageHandler::MuJoCoMessageHandler(mj::Simulate *sim)
      : Node("MuJoCoMessageHandler"), sim_(sim), name_prefix("simulation/")
  {
    model_param_name = name_prefix + "model_file";
    this->declare_parameter(model_param_name, "");

    reset_service_ = this->create_service<communication::srv::SimulationReset>(
        name_prefix + "sim_reset",
        std::bind(&MuJoCoMessageHandler::reset_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        name_prefix + "imu_data", qos);
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        name_prefix + "joint_states", qos);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        name_prefix + "odom", qos);

    timers_.emplace_back(this->create_wall_timer(
        2.5ms, std::bind(&MuJoCoMessageHandler::imu_callback, this)));
    timers_.emplace_back(this->create_wall_timer(
        1ms, std::bind(&MuJoCoMessageHandler::joint_callback, this)));
    timers_.emplace_back(this->create_wall_timer(
        20ms, std::bind(&MuJoCoMessageHandler::odom_callback, this)));
    timers_.emplace_back(this->create_wall_timer(
        100ms, std::bind(&MuJoCoMessageHandler::drop_old_message, this)));

    actuator_cmd_subscription_ =
        this->create_subscription<communication::msg::ActuatorCmds>(
            name_prefix + "actuators_cmds", qos,
            std::bind(&MuJoCoMessageHandler::actuator_cmd_callback, this,
                      std::placeholders::_1));

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_handle_ = param_subscriber_->add_parameter_callback(
        model_param_name, std::bind(&MuJoCoMessageHandler::parameter_callback,
                                    this, std::placeholders::_1));

    actuator_cmds_ptr_ = std::make_shared<ActuatorCmds>();

    RCLCPP_INFO(this->get_logger(), "Start MuJoCoMessageHandler ...");

    std::string model_file = this->get_parameter(model_param_name)
                                 .get_parameter_value()
                                 .get<std::string>();
    mju::strcpy_arr(sim_->filename, model_file.c_str());
    sim_->uiloadrequest.fetch_add(1);
  }

  MuJoCoMessageHandler::~MuJoCoMessageHandler()
  {
    RCLCPP_INFO(this->get_logger(), "close node ...");
  }

  void MuJoCoMessageHandler::reset_callback(
      const std::shared_ptr<communication::srv::SimulationReset::Request> request,
      std::shared_ptr<communication::srv::SimulationReset::Response> response)
  {
    while (sim_->d_ == nullptr && rclcpp::ok())
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (sim_->d_ != nullptr)
    {
      if (request->header.frame_id != std::string(&sim_->m_->names[0]))
      {
        RCLCPP_ERROR(this->get_logger(), "reset request is not for %s",
                     &sim_->m_->names[0]);
        response->is_success = false;
      }
      else
      {
        sim_->mtx.lock();
        mj_resetData(sim_->m_, sim_->d_);
        sim_->d_->qpos[0] = request->base_pose.position.x;
        sim_->d_->qpos[1] = request->base_pose.position.y;
        sim_->d_->qpos[2] = request->base_pose.position.z;
        sim_->d_->qpos[3] = request->base_pose.orientation.w;
        sim_->d_->qpos[4] = request->base_pose.orientation.x;
        sim_->d_->qpos[5] = request->base_pose.orientation.y;
        sim_->d_->qpos[6] = request->base_pose.orientation.z;

        for (int i = 0; i < request->joint_state.position.size(); i++)
        {
          int joint_id = mj_name2id(sim_->m_, mjOBJ_JOINT,
                                    request->joint_state.name[i].c_str());
          if (joint_id > -1)
          {
            sim_->d_->qpos[sim_->m_->jnt_qposadr[joint_id]] =
                request->joint_state.position[i];
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Request joint %s does not exist",
                        request->joint_state.name[i].c_str());
          }
        }
        for (size_t k = 0; k < actuator_cmds_ptr_->actuators_name.size(); k++)
        {
          actuator_cmds_ptr_->kp[k] = 0;
          actuator_cmds_ptr_->pos[k] = 0;
          actuator_cmds_ptr_->kd[k] = 0;
          actuator_cmds_ptr_->vel[k] = 0;
          actuator_cmds_ptr_->torque[k] = 0.0;
        }
        sim_->mtx.unlock();
        response->is_success = true;
        RCLCPP_INFO(this->get_logger(), "reset robot state...");
        RCLCPP_INFO(this->get_logger(), "robot total mass: %f",
                    sim_->m_->body_subtreemass[0]);
      }
    }
    else
    {
      response->is_success = false;
    }
  }

  void MuJoCoMessageHandler::imu_callback()
  {
    if (sim_->d_ != nullptr)
    {
      auto message = sensor_msgs::msg::Imu();
      message.header.frame_id = &sim_->m_->names[0];
      message.header.stamp = rclcpp::Clock().now();
      const std::lock_guard<std::recursive_mutex> lock(sim_->mtx);

      for (int i = 0; i < sim_->m_->nsensor; i++)
      {
        if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER)
        {
          message.linear_acceleration.x =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
          message.linear_acceleration.y =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
          message.linear_acceleration.z =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
        }
        else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_FRAMEQUAT)
        {
          message.orientation.w = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
          message.orientation.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
          message.orientation.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
          message.orientation.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 3];
        }
        else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_GYRO)
        {
          message.angular_velocity.x =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
          message.angular_velocity.y =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
          message.angular_velocity.z =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
        }
      }
      imu_publisher_->publish(message);
    }
  }

  void MuJoCoMessageHandler::odom_callback()
  {
    const std::lock_guard<std::recursive_mutex> lock(sim_->mtx);
    if (sim_->d_ != nullptr)
    {
      auto message = nav_msgs::msg::Odometry();
      message.header.frame_id = &sim_->m_->names[0];
      message.header.stamp = rclcpp::Clock().now();
      message.pose.pose.position.x = sim_->d_->qpos[0];
      message.pose.pose.position.y = sim_->d_->qpos[1];
      message.pose.pose.position.z = sim_->d_->qpos[2];
      message.pose.pose.orientation.w = sim_->d_->qpos[3];
      message.pose.pose.orientation.x = sim_->d_->qpos[4];
      message.pose.pose.orientation.y = sim_->d_->qpos[5];
      message.pose.pose.orientation.z = sim_->d_->qpos[6];
      message.twist.twist.linear.x = sim_->d_->qvel[0];
      message.twist.twist.linear.y = sim_->d_->qvel[1];
      message.twist.twist.linear.z = sim_->d_->qvel[2];
      message.twist.twist.angular.x = sim_->d_->qvel[3];
      message.twist.twist.angular.y = sim_->d_->qvel[4];
      message.twist.twist.angular.z = sim_->d_->qvel[5];
      odom_publisher_->publish(message);
    }
  }

  void MuJoCoMessageHandler::joint_callback()
  {
    const std::lock_guard<std::recursive_mutex> lock(sim_->mtx);

    if (sim_->d_ != nullptr)
    {
      sensor_msgs::msg::JointState jointState;
      jointState.header.frame_id = &sim_->m_->names[0];
      jointState.header.stamp = rclcpp::Clock().now();
      for (int i = 0; i < sim_->m_->njnt; i++)
      {
        if (sim_->m_->jnt_type[i] == mjtJoint::mjJNT_HINGE)
        {
          std::string jnt_name(mj_id2name(sim_->m_, mjtObj::mjOBJ_JOINT, i));
          jointState.name.emplace_back(jnt_name);
          jointState.position.push_back(sim_->d_->qpos[sim_->m_->jnt_qposadr[i]]);
          jointState.velocity.push_back(sim_->d_->qvel[sim_->m_->jnt_dofadr[i]]);
          jointState.effort.push_back(
              sim_->d_->qfrc_actuator[sim_->m_->jnt_dofadr[i]]);
        }
      }
      joint_state_publisher_->publish(jointState);
    }
  }

  void MuJoCoMessageHandler::actuator_cmd_callback(
      const communication::msg::ActuatorCmds::SharedPtr msg) const
  {
    if (sim_->d_ != nullptr)
    {
      actuator_cmds_ptr_->time = this->now().seconds();
      actuator_cmds_ptr_->actuators_name.resize(msg->actuators_name.size());
      actuator_cmds_ptr_->kp.resize(msg->kp.size());
      actuator_cmds_ptr_->pos.resize(msg->pos.size());
      actuator_cmds_ptr_->kd.resize(msg->kd.size());
      actuator_cmds_ptr_->vel.resize(msg->vel.size());
      actuator_cmds_ptr_->torque.resize(msg->torque.size());
      for (size_t k = 0; k < msg->actuators_name.size(); k++)
      {
        actuator_cmds_ptr_->actuators_name[k] = msg->actuators_name[k];
        actuator_cmds_ptr_->kp[k] = msg->kp[k];
        actuator_cmds_ptr_->pos[k] = msg->pos[k];
        actuator_cmds_ptr_->kd[k] = msg->kd[k];
        actuator_cmds_ptr_->vel[k] = msg->vel[k];
        actuator_cmds_ptr_->torque[k] = msg->torque[k];
      }
      // RCLCPP_INFO(this->get_logger(), "subscribe actuator cmds");
    }
  }

  void MuJoCoMessageHandler::parameter_callback(const rclcpp::Parameter &)
  {
    std::string model_file = this->get_parameter(model_param_name)
                                 .get_parameter_value()
                                 .get<std::string>();
    RCLCPP_INFO(this->get_logger(), "load model from: %s", model_file.c_str());
    mju::strcpy_arr(sim_->filename, model_file.c_str());
    sim_->uiloadrequest.fetch_add(1);
  }

  void MuJoCoMessageHandler::drop_old_message()
  {
    if (abs(actuator_cmds_ptr_->time - this->now().seconds()) > 0.2)
    {
      for (size_t k = 0; k < actuator_cmds_ptr_->actuators_name.size(); k++)
      {
        actuator_cmds_ptr_->kp[k] = 0.0;
        actuator_cmds_ptr_->pos[k] = 0.0;
        actuator_cmds_ptr_->kd[k] = 1.0;
        actuator_cmds_ptr_->vel[k] = 0.0;
        actuator_cmds_ptr_->torque[k] = 0.0;
      }
    }
  }

  std::shared_ptr<MuJoCoMessageHandler::ActuatorCmds>
  MuJoCoMessageHandler::get_actuator_cmds_ptr()
  {
    return actuator_cmds_ptr_;
  }

} // namespace breflex
