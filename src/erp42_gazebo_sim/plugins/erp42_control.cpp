/**
 * -------------------------------------------------------------------------------------------------
 * 
 * Copyright 2025 Minkyu Kil
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * @file    erp42_control.cpp
 * @brief   ERP42 gazebo-sim control plugin
 * @author  Minkyu Kil
 * @date    2026-03-18
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gazebo_sim/erp42_control.hpp"
#include "erp42_util/erp42_parameters.hpp"
#include "erp42_msgs/msg/control_command.hpp"
#include "erp42_msgs/msg/feedback.hpp"
#include "erp42_msgs/srv/mode_command.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gz/common/Profiler.hh"
#include "gz/math/SpeedLimiter.hh"
#include "gz/plugin/Register.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <set>

using namespace gz;
using namespace sim;
using namespace systems;

/** @brief Private implementation for ERP42Control */
class gz::sim::systems::ERP42ControlPrivate
{
public:

    /**
     * @brief Callback function for control command subscription
     * @param[in] msg The received control command message
     * @details Updates target speed, steering, and brake based on the received command,
     * while respecting emergency stop and manual mode conditions. 
     * Uses mutex to protect against data races.
     */
    void control_command_callback(const erp42_msgs::msg::ControlCommand::ConstSharedPtr &msg);

    /**
     * @brief Callback function for mode command service
     * @param[in] request The received mode command request containing emergency stop, 
     * manual mode, and gear information
     * @param[in] response The response to be sent back to the service caller, indicating success
     * @details Updates emergency stop, manual mode, and gear state based on the received request.
     * Uses mutex to protect against data races.
     */
    void mode_command_callback(
        const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
        erp42_msgs::srv::ModeCommand::Response::SharedPtr response
    );

    /**
     * @brief Updates odometry information and publishes it 
     * if the publish_odometry option is enabled
     * @param[in] _info UpdateInfo containing timing and state information
     * @param[in] _ecm The EntityComponentManager for interacting with the simulation
     * @details Retrieves the current pose and velocity of the vehicle, 
     * constructs an odometry message, and publishes it if the publish_odometry option is enabled.
     * Also broadcasts the corresponding transform for the odometry frame.
     */
    void update_odometry(const UpdateInfo &_info, const EntityComponentManager &_ecm);

    /**
     * @brief Updates feedback information and publishes it at a fixed rate
     * @param[in] _info UpdateInfo containing timing and state information
     * @param[in] _ecm The EntityComponentManager for interacting with the simulation
     * @details Retrieves the current joint positions and velocities, computes feedback information
     * such as steering angle and speed, and publishes it at a fixed rate defined by feedback_period.
     */
    void update_feedback(const UpdateInfo &_info, const EntityComponentManager &_ecm);

    /**
     * @brief Updates control commands and applies them to the vehicle
     * @param[in] _info UpdateInfo containing timing and state information
     * @param[in] _ecm The EntityComponentManager for interacting with the simulation
     * @details Retrieves the current control commands, applies speed and steering limiters,
     * and updates the vehicle's joint velocities accordingly.
     */
    void update_control(const UpdateInfo &_info, const EntityComponentManager &_ecm);

    // Mutex to protect data race from callback function
    std::mutex mutex;

    // Gazebo sim model
    Model model {kNullEntity};

    // Gazebo sim model canonical link
    Link canonical_link {kNullEntity};

    // Speed limiters
    std::unique_ptr<math::SpeedLimiter> velocity_limiter;
    std::unique_ptr<math::SpeedLimiter> steering_limiter;
    std::unique_ptr<math::SpeedLimiter> brake_limiter;

    // Wheel joint name data containers
    enum JointName
    {
        LEFT_STEER,
        RIGHT_STEER,
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT,
        JOINT_NUM
        
    }; // enum JointName

    // Joint entity container
    std::array<Entity, JOINT_NUM> joint_entities {};

    // Joint name container
    std::array<std::string, JOINT_NUM> joint_names {};

    // Joint target velocity conatiner
    std::array<double, JOINT_NUM> joint_velocities {};

    // Vehicle geometry
    double wheel_base         {1.040};
    double kingpin_width      {0.985};
    double front_wheel_tread  {0.985};
    double rear_wheel_tread   {0.985};
    double front_wheel_radius {0.270};
    double rear_wheel_radius  {0.270};

    // Vehicle dynamics limits
    double steer_p_gain       {1.0};
    double steer_limit        {0.43633231299};
    double velocity_limit     {7.0};
    double acceleration_limit {1.5};
    double brake_deceleration {10.0};

    // Control elements
    double target_speed       {0.0};
    double target_steering    {0.0};
    std::uint8_t target_brake {150};

    // Control element history for limiters
    double last_velocity_command0;
    double last_velocity_command1;
    double last_steering_command0;
    double last_steering_command1;

    // Mode commands
    bool emergency_stop {true};
    bool manual_mode    {true};
    std::uint8_t gear   {1};

    // Feedback data history
    std::chrono::steady_clock::duration feedback_period {std::chrono::milliseconds(20)};
    std::chrono::steady_clock::duration last_feedback_publication_time;
    double prev_encoder_angle  {0.0};
    std::int32_t encoder_count {0};
    std::uint8_t heartbeat     {0};

    // Odometry publish options
    bool publish_odometry {false};
    double odometry_frequency;
    std::chrono::steady_clock::duration odometry_period;
    std::chrono::steady_clock::duration last_odometry_publication_time;

    // Odometry topic parameters
    std::string odometry_topic;
    std::string odometry_frame;
    std::string child_frame;

    // ROS2 node
    rclcpp::Node::SharedPtr node;

    // Transform broadcaster for odometry frame
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_transform_broadcaster;

    // Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;

    // Feedback publisher
    rclcpp::Publisher<erp42_msgs::msg::Feedback>::SharedPtr feedback_pub;

    // Control command publisher
    rclcpp::Subscription<erp42_msgs::msg::ControlCommand>::SharedPtr control_command_sub;

    // Mode command service server
    rclcpp::Service<erp42_msgs::srv::ModeCommand>::SharedPtr mode_command_srv;

}; // class gz::sim::systems::ERP42ControlPrivate

/**
 * @brief Constructor for ERP42Control plugin
 * @details Initializes internal state and prepares for configuration
 */
ERP42Control::ERP42Control() : dataPtr(std::make_unique<ERP42ControlPrivate>())
{
}

/**
 * @brief Configure plugin based on SDF parameters and initializes ROS2 interfaces
 * @param[in] _entity The entity associated with the plugin (the ERP42 model)
 * @param[in] _sdf The SDF element containing plugin parameters
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @param[in] _eventMgr The EventManager for subscribing to simulation events
 * @details Parses SDF parameters, sets up ROS2 publishers/subscribers/services, 
 * and initializes internal state
 */
void ERP42Control::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /* _eventMgr */
)
{
    // Initialize model and canonical link
    this->dataPtr->model = Model(_entity);

    // Check if the model entity is valid
    if(!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << 
            "ERP42Control should be attached to a model entity. " << 
            "Failed to initialize." << 
        std::endl;

        return;
    }

    // Find the canonical link of the model
    std::vector<Entity> links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(),
        components::CanonicalLink()
    );
    if(!links.empty())
    {
        this->dataPtr->canonical_link = Link(links[0]);
    }

    // Parse joint names from SDF, using default values if not specified
    this->dataPtr->joint_names[this->dataPtr->LEFT_STEER] = 
    _sdf->Get<std::string>("left_steer_joint", "left_steer_joint").first;

    this->dataPtr->joint_names[this->dataPtr->RIGHT_STEER] =  
    _sdf->Get<std::string>("right_steer_joint", "right_steer_joint").first;

    this->dataPtr->joint_names[this->dataPtr->FRONT_LEFT] =  
    _sdf->Get<std::string>("front_left_joint", "front_left_joint").first;

    this->dataPtr->joint_names[this->dataPtr->FRONT_RIGHT] =  
    _sdf->Get<std::string>("front_right_joint", "front_right_joint").first;

    this->dataPtr->joint_names[this->dataPtr->REAR_LEFT] =  
    _sdf->Get<std::string>("rear_left_joint", "rear_left_joint").first;

    this->dataPtr->joint_names[this->dataPtr->REAR_RIGHT] =  
    _sdf->Get<std::string>("rear_right_joint", "rear_right_joint").first;

    // Parse vehicle geometry parameters from SDF, using default values if not specified
    this->dataPtr->wheel_base = 
    _sdf->Get<double>("wheel_base", this->dataPtr->wheel_base).first;

    this->dataPtr->kingpin_width = 
    _sdf->Get<double>("kingpin_width", this->dataPtr->kingpin_width).first;

    this->dataPtr->front_wheel_tread = 
    _sdf->Get<double>("front_wheel_tread", this->dataPtr->front_wheel_tread).first;

    this->dataPtr->rear_wheel_tread = 
    _sdf->Get<double>("rear_wheel_tread", this->dataPtr->rear_wheel_tread).first;

    this->dataPtr->front_wheel_radius = 
    _sdf->Get<double>("front_wheel_radius", this->dataPtr->front_wheel_radius).first;

    this->dataPtr->rear_wheel_radius = 
    _sdf->Get<double>("rear_wheel_radius", this->dataPtr->rear_wheel_radius).first;

    // Initialize speed limiters
    this->dataPtr->velocity_limiter = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->steering_limiter = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->brake_limiter    = std::make_unique<math::SpeedLimiter>();

    // Parse control limits from SDF, using default values if not specified
    this->dataPtr->brake_deceleration = _sdf->Get<double>("brake_deceleration", 10.0).first;

    this->dataPtr->steer_p_gain = 
    _sdf->Get<double>("steering_p_gain", this->dataPtr->steer_p_gain).first;

    this->dataPtr->steer_limit = 
    _sdf->Get<double>("steering_limit", this->dataPtr->steer_limit).first;

    this->dataPtr->steering_limiter->SetMaxVelocity(this->dataPtr->steer_limit);
    this->dataPtr->steering_limiter->SetMinVelocity(-this->dataPtr->steer_limit);

    if(_sdf->HasElement("velocity_limit"))
    {
        this->dataPtr->velocity_limit = _sdf->Get<double>("velocity_limit");
        this->dataPtr->velocity_limiter->SetMaxVelocity(this->dataPtr->velocity_limit);
        this->dataPtr->velocity_limiter->SetMinVelocity(-this->dataPtr->velocity_limit);
    }

    if(_sdf->HasElement("acceleration_limit"))
    {
        this->dataPtr->acceleration_limit = _sdf->Get<double>("acceleration_limit");
        this->dataPtr->velocity_limiter->SetMaxAcceleration(this->dataPtr->acceleration_limit);
        this->dataPtr->steering_limiter->SetMaxAcceleration(this->dataPtr->acceleration_limit);
        this->dataPtr->velocity_limiter->SetMinAcceleration(-this->dataPtr->acceleration_limit);
        this->dataPtr->steering_limiter->SetMinAcceleration(-this->dataPtr->acceleration_limit);
    }
    
    // Parse odometry publish options from SDF, using default values if not specified
    this->dataPtr->publish_odometry = _sdf->Get<bool>("publish_odometry", false).first;
    this->dataPtr->odometry_topic   = _sdf->Get<std::string>("odometry_topic", "odom").first;
    this->dataPtr->odometry_frame   = _sdf->Get<std::string>("odometry_frame", "odom").first;
    this->dataPtr->child_frame      = _sdf->Get<std::string>("child_frame", "base_link").first;

    this->dataPtr->odometry_frequency = _sdf->Get<double>("odometry_frequency", 50).first;
    if(0 < this->dataPtr->odometry_frequency)
    {
        this->dataPtr->odometry_period = 
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / this->dataPtr->odometry_frequency)
        );
    }

    // Initialize ROS2 node and interfaces
    if(!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    this->dataPtr->node = std::make_shared<rclcpp::Node>(
        "erp42_gazebo_control",
        rclcpp::NodeOptions().parameter_overrides(
            {rclcpp::Parameter("use_sim_time", true)}
        )
    );
    
    // Set up ROS2 publishers and subscribers
    if(this->dataPtr->publish_odometry)
    {
        this->dataPtr->odom_transform_broadcaster = 
        std::make_shared<tf2_ros::TransformBroadcaster>(this->dataPtr->node);

        this->dataPtr->odometry_pub = 
        this->dataPtr->node->create_publisher<nav_msgs::msg::Odometry>(
            this->dataPtr->odometry_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
        );
    }

    this->dataPtr->feedback_pub = 
    this->dataPtr->node->create_publisher<erp42_msgs::msg::Feedback>(
        "/erp42/feedback",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
    );

    this->dataPtr->control_command_sub = 
    this->dataPtr->node->create_subscription<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [this](const erp42_msgs::msg::ControlCommand::ConstSharedPtr & msg) 
        {
            this->dataPtr->control_command_callback(msg);
        }
    );

    this->dataPtr->mode_command_srv = 
    this->dataPtr->node->create_service<erp42_msgs::srv::ModeCommand>(
        "/erp42/mode_command",
        [this](
            const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
            erp42_msgs::srv::ModeCommand::Response::SharedPtr response
        )
        {
            this->dataPtr->mode_command_callback(request, response);
        }
    );

    this->dataPtr->node->declare_parameter<std::string>("port_path", "/dev/ttyUSB0");
    this->dataPtr->node->declare_parameter<int>("baud_rate", 115200);

    this->dataPtr->node->declare_parameter<double>(
        "max_speed_mps", 
        this->dataPtr->velocity_limit
    );

    this->dataPtr->node->declare_parameter<double>(
        "max_steering_deg", 
        this->dataPtr->steer_limit * 180.0 / M_PI
    );

    this->dataPtr->node->declare_parameter<double>(
        "steering_offset_deg", 0.0
    );
}

void ERP42Control::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42Control::PreUpdate");

    // Check for time jump and warn if detected, 
    // as it may cause issues with the plugin's operation
    if(_info.dt < std::chrono::steady_clock::duration::zero())
    {
        gzwarn << 
            "Detected jump back in time [" << 
            std::chrono::duration<double>(_info.dt).count() << 
            "s]. System may not work properly." << 
        std::endl;
    }

    // Check if joint entities have been found, and if not, attempt to find them again.
    const bool null_entity_found = std::any_of(
        this->dataPtr->joint_entities.begin(),
        this->dataPtr->joint_entities.end(),
        [](Entity e){ return e == kNullEntity; }
    );

    // If any joint entity is still null, attempt to find them again. 
    // This can happen if the plugin is loaded before the model is fully initialized.
    if(null_entity_found)
    {
        // Get the model name for logging purposes
        const auto model_name = this->dataPtr->model.Name(_ecm);

        // Use a static set to track which models have already been warned about missing joints,
        // to avoid spamming warnings for the same model on every update.
        bool warned = false;
        static std::set<std::string> warned_models;

        // Try to find joint entities again
        for(std::uint8_t idx = 0; idx < this->dataPtr->JOINT_NUM; idx++)
        {
            const std::string joint_name = this->dataPtr->joint_names[idx];
            const Entity joint_entity = this->dataPtr->model.JointByName(_ecm, joint_name);

            // If the joint entity is found, store it in the joint_entities array.
            if(joint_entity != kNullEntity)
            {
                this->dataPtr->joint_entities[idx] = joint_entity;
            }
            else
            {
                gzwarn << 
                    "Failed to find joint [" << joint_name << "] " <<
                    "for model [" << model_name << "]" <<
                std::endl;

                warned = true;
            }
        }

        // If we failed to find any joints,
        // and we haven't already warned about this model, log a warning.
        if(warned)
        {
            warned_models.insert(model_name);
            return;
        }

        // If we successfully found all joints, and we had previously warned about this model, 
        // log a message and remove it from the warned set.
        if(warned_models.find(model_name) != warned_models.end())
        {
            gzmsg << 
                "Found joints for model [" << model_name << "], " << 
                "plugin will start working." << 
            std::endl;

            warned_models.erase(model_name);
        }
    }

    // If the simulation is paused, 
    // do not update joint commands or create necessary components.
    if(_info.paused)
    {
        return;
    }

    // Set joint velocity commands based on the current target velocities.
    for(std::uint8_t idx = 0; idx < this->dataPtr->JOINT_NUM; idx++)
    {
        _ecm.SetComponentData<components::JointVelocityCmd>(
            this->dataPtr->joint_entities[idx],
            {this->dataPtr->joint_velocities[idx]}
        );
    }

    // Ensure that necessary components for joints and canonical link exist,
    // creating them if they are missing. This is important for the plugin to function correctly,
    // especially if the plugin is loaded before the model is fully initialized.
    for(std::uint8_t idx = 0; idx < this->dataPtr->JOINT_NUM; idx++)
    {
        // Check if JointPosition component exists for the joint, 
        // and create it if it doesn't
        auto joint_position = _ecm.Component<components::JointPosition>(
            this->dataPtr->joint_entities[idx]
        );

        // If the JointPosition component is missing, create it. 
        // This is necessary for the plugin to function correctly,
        if(!joint_position)
        {
            _ecm.CreateComponent(
                this->dataPtr->joint_entities[idx], 
                components::JointPosition()
            );
        }

        // Check if JointVelocity component exists for the joint,
        // and create it if it doesn't
        auto joint_velocity = _ecm.Component<components::JointVelocity>(
            this->dataPtr->joint_entities[idx]
        );

        // If the JointVelocity component is missing, create it.
        if(!joint_velocity)
        {
            _ecm.CreateComponent(
                this->dataPtr->joint_entities[idx],
                components::JointVelocity()
            );
        }
    }

    // Ensure that WorldPose, WorldLinearVelocity, and WorldAngularVelocity components exist 
    // for the canonical link, creating them if they are missing. 
    // This is important for the plugin to function correctly, 
    // especially if the plugin is loaded before the model is fully initialized.
    if(!_ecm.Component<components::WorldPose>(this->dataPtr->canonical_link.Entity()))
    {
        _ecm.CreateComponent(
            this->dataPtr->canonical_link.Entity(), 
            components::WorldPose()
        );
    }
    if(!_ecm.Component<components::WorldLinearVelocity>(this->dataPtr->canonical_link.Entity()))
    {
        _ecm.CreateComponent(
            this->dataPtr->canonical_link.Entity(), 
            components::WorldLinearVelocity()
        );
    }
    if(!_ecm.Component<components::WorldAngularVelocity>(this->dataPtr->canonical_link.Entity()))
    {
        _ecm.CreateComponent(
            this->dataPtr->canonical_link.Entity(),
            components::WorldAngularVelocity()
        );
    }
}

/**
 * @brief Post-update function for ERP42Control plugin, called after each simulation update
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Handles ROS2 communication, updates odometry and feedback information, 
 * and applies control commands
 */
void ERP42Control::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42Control::PostUpdate");

    // If the simulation is paused, do not update odometry, feedback, or control.
    if(_info.paused)
    {
        return;
    }

    // Check if joint entities have been found, and if not, do not proceed with updates.
    const bool null_entity_found = std::any_of(
        this->dataPtr->joint_entities.begin(),
        this->dataPtr->joint_entities.end(),
        [](Entity e){ return e == kNullEntity; }
    );

    // If any joint entity is still null, do not proceed with updates, 
    // as the plugin is not fully initialized.
    if(null_entity_found)
    {
        return;
    }

    // Process ROS2 callbacks to handle incoming messages and service requests.
    rclcpp::spin_some(dataPtr->node);

    // Update odometry information and publish it if enabled.
    if(this->dataPtr->publish_odometry)
    {
        this->dataPtr->update_odometry(_info, _ecm);
    }

    // Update feedback information and publish it at a fixed rate.
    this->dataPtr->update_feedback(_info, _ecm);

    // Update control commands and apply them to the vehicle.
    this->dataPtr->update_control(_info, _ecm);
}

/**
 * @brief Updates odometry information and publishes it if the publish_odometry option is enabled
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current pose and velocity of the vehicle, constructs an odometry message,
 * and publishes it if the publish_odometry option is enabled. 
 * Also broadcasts the corresponding transform for the odometry frame.
 */
void ERP42ControlPrivate::update_odometry(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42Control::update_odometry");

    // Check if it's time to publish the next odometry message based on the defined frequency.
    if((_info.simTime - this->last_odometry_publication_time) < odometry_period)
    {
        return;
    }

    // Update the last publication time to the current simulation time.
    this->last_odometry_publication_time = _info.simTime;

    // Retrieve the current pose, linear velocity, and angular velocity of the canonical link.
    auto pose = this->canonical_link.WorldPose(_ecm);
    auto linear_velocity = this->canonical_link.WorldLinearVelocity(_ecm);
    auto angular_velocity = this->canonical_link.WorldAngularVelocity(_ecm);

    if(!pose || !linear_velocity || !angular_velocity)
    {
        return;
    }

    // Convert simulation time to ROS2 time for message timestamps.
    const rclcpp::Time sim_stamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count()
    );

    // Construct the odometry message with the current pose and velocity information.
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp    = sim_stamp;
    odometry.header.frame_id = odometry_frame;
    odometry.child_frame_id  = child_frame;
    
    odometry.pose.pose.position.x    = pose->Pos().X();
    odometry.pose.pose.position.y    = pose->Pos().Y();
    odometry.pose.pose.position.z    = pose->Pos().Z();
    odometry.pose.pose.orientation.x = pose->Rot().X();
    odometry.pose.pose.orientation.y = pose->Rot().Y();
    odometry.pose.pose.orientation.z = pose->Rot().Z();
    odometry.pose.pose.orientation.w = pose->Rot().W();

    odometry.twist.twist.linear.x  = linear_velocity->X();
    odometry.twist.twist.linear.y  = linear_velocity->Y();
    odometry.twist.twist.linear.z  = linear_velocity->Z();
    odometry.twist.twist.angular.x = angular_velocity->X();
    odometry.twist.twist.angular.y = angular_velocity->Y();
    odometry.twist.twist.angular.z = angular_velocity->Z();

    // Broadcast the transform corresponding to the odometry frame for use in ROS2 TF.
    geometry_msgs::msg::TransformStamped transform;
    transform.header                  = odometry.header;
    transform.child_frame_id          = odometry.child_frame_id;
    transform.transform.translation.x = pose->Pos().X();
    transform.transform.translation.y = pose->Pos().Y();
    transform.transform.translation.z = pose->Pos().Z();
    transform.transform.rotation      = odometry.pose.pose.orientation;

    // Publish the odometry message and broadcast the transform.
    this->odometry_pub->publish(odometry);
    this->odom_transform_broadcaster->sendTransform(transform);
}

/**
 * @brief Updates feedback information and publishes it at a fixed rate
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current joint positions and velocities, computes feedback information
 * such as steering angle and speed, and publishes it at a fixed rate defined by feedback_period.
 */
void ERP42ControlPrivate::update_feedback(
    const UpdateInfo &_info, 
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42Control::update_feedback");

    // Check if it's time to publish the next feedback message based on the defined frequency.
    if((_info.simTime - this->last_feedback_publication_time) < feedback_period)
    {
        return;
    }

    // Update the last publication time to the current simulation time.
    this->last_feedback_publication_time = _info.simTime;

    // Retrieve joint positions and velocities for steering and wheel joints.
    const auto left_steer_joint_position = 
    _ecm.Component<components::JointPosition>(this->joint_entities[LEFT_STEER]);

    const auto right_steer_joint_position = 
    _ecm.Component<components::JointPosition>(this->joint_entities[RIGHT_STEER]);

    // Wheel position : using actual encoder
    const auto front_left_wheel_position = 
    _ecm.Component<components::JointPosition>(this->joint_entities[FRONT_LEFT]);

    // Wheel velocity : using actual encoder
    const auto front_left_wheel_velocity = 
    _ecm.Component<components::JointVelocity>(this->joint_entities[FRONT_LEFT]);

    if(
        !left_steer_joint_position  || left_steer_joint_position->Data().empty()  ||
        !right_steer_joint_position || right_steer_joint_position->Data().empty() ||
        !front_left_wheel_position  || front_left_wheel_position->Data().empty()  ||
        !front_left_wheel_velocity  || front_left_wheel_velocity->Data().empty()
    )
    {
        return;
    }

    // Convert simulation time to ROS2 time for message timestamps.
    const rclcpp::Time sim_stamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count()
    );
    erp42_msgs::msg::Feedback feedback;
    feedback.header.stamp = sim_stamp;
    feedback.header.frame_id = child_frame;

    // Set emergency stop
    feedback.emergency_stop = this->emergency_stop;

    // Set manual mode
    feedback.manual_mode = this->manual_mode;
    
    // Set gear
    feedback.gear = this->gear;

    // Set steering (must be computed before speed for Ackermann correction)
    const double tan_left  = std::tan(left_steer_joint_position->Data()[0]);
    const double tan_right = std::tan(right_steer_joint_position->Data()[0]);
    const double tan_sum   = tan_left + tan_right;
    feedback.steering = (std::abs(tan_sum) > 1e-6) ? 
        std::atan(2.0 * tan_left * tan_right / tan_sum) : 0.0;

    // Set speed with Ackermann correction
    const double fl_factor = 
        1.0 - (front_wheel_tread * std::tan(feedback.steering)) / (2.0 * wheel_base);

    const double fl_wheel_speed =
        front_left_wheel_velocity->Data()[0] * front_wheel_radius;

    feedback.speed = (std::abs(fl_factor) > 0.01) ? (fl_wheel_speed / fl_factor) : fl_wheel_speed;

    // Set brake
    feedback.brake = this->target_brake;

    // Update encoder count
    const double encoder_angle = front_left_wheel_position->Data()[0];
    const double delta_encoder_angle = 
    std::remainder(encoder_angle - prev_encoder_angle, 2.0 * M_PI);

    // Compute encoder position difference
    if((2.0 * M_PI / erp42_util::ENCODER_CPR) < std::abs(delta_encoder_angle))
    {
        // Compute encoder count
        this->encoder_count += static_cast<int>(
            delta_encoder_angle / (2.0 * M_PI) * erp42_util::ENCODER_CPR
        );

        // Update history
        prev_encoder_angle = encoder_angle;
    }

    // Set encoder count
    feedback.encoder_count = this->encoder_count;

    // Set heartbeat
    feedback.heartbeat = this->heartbeat++;

    // Publish feedback message
    this->feedback_pub->publish(feedback);
}

/**
 * @brief Updates control commands and applies them to the vehicle
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current control commands, applies speed and steering limiters,
 * and updates the vehicle's joint velocities accordingly.
 */
void ERP42ControlPrivate::control_command_callback(
    const erp42_msgs::msg::ControlCommand::ConstSharedPtr &msg
)
{
    // Lock the mutex to protect against data
    std::lock_guard<std::mutex> lock(this->mutex);

    // Update target speed, steering, and brake from the received control command message,
    // applying limits to ensure they are within the defined constraints.
    this->target_speed = std::clamp(msg->speed, 0.0, this->velocity_limit);

    // Steering command is clamped within the defined steering limits 
    // to prevent excessive steering angles.
    this->target_steering = std::clamp(msg->steering, -this->steer_limit, this->steer_limit);

    // Brake command is clamped between 0 and 150, where 150 represents full braking force.
    this->target_brake = std::clamp<std::uint8_t>(msg->brake, 0, 150);
}

/**
 * @brief Service callback for mode commands, 
 * allowing external control over emergency stop, manual mode, and gear state
 */
void ERP42ControlPrivate::mode_command_callback(
    const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
    erp42_msgs::srv::ModeCommand::Response::SharedPtr response
)
{
    // Lock the mutex to protect against concurrent access to mode state variables,
    // ensuring thread safety when processing mode command service requests.
    std::lock_guard<std::mutex> lock(this->mutex);

    // If emergency stop is activated, 
    // it will override manual mode and gear settings in the control update.
    this->emergency_stop = request->emergency_stop;

    // Update mode states based on the received service request.
    this->manual_mode = request->manual_mode;

    // Update gear state based on the received service request.
    this->gear = request->gear;

    // Set the response success flag to true to indicate 
    // that the mode command was processed successfully.
    response->success = true;
}

/**
 * @brief Updates control commands and applies them to the vehicle
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current control commands, applies speed and steering limiters,
 * and updates the vehicle's joint velocities accordingly.
 * If emergency stop or manual mode is active, it will override the target speed, steering,
 * and brake commands to bring the vehicle to a safe state.
 */
void ERP42ControlPrivate::update_control(
    const UpdateInfo &_info, 
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42Control::update_control");

    // Retrieve the current mode states and control commands,
    // while holding the mutex to ensure thread safety.
    bool emergency_stop_on, manual_mode_on;
    std::uint8_t gear_mode, brake;
    double velocity, steering;
    {
        std::lock_guard<std::mutex> lock(this->mutex);

        emergency_stop_on = this->emergency_stop;
        manual_mode_on = this->manual_mode;
        gear_mode = this->gear;

        velocity = this->target_speed;
        steering = this->target_steering;
        brake = this->target_brake;
    }

    // If emergency stop or manual mode is active, 
    // override the control commands to bring the vehicle to a safe state.
    if(emergency_stop_on || manual_mode_on)
    {
        velocity = 0.0;
        steering = 0.0;
        brake = 150;
    }

    // Apply gear direction to the velocity command.
    switch(gear_mode)
    {
    case 0: // DRIVE
        velocity= std::abs(velocity);
        break;
    case 2: // REVERSE
        velocity = -std::abs(velocity);
        break;
    default: // NEUTRAL or other modes
        velocity = 0.0;
        break;
    }

    // Apply brake scaling to the velocity and steering commands.
    const double brake_scale = static_cast<double>(brake) / 150.0;

    // If brake is applied, use the brake limiter to decelerate the vehicle smoothly,
    // otherwise use the velocity limiter to apply acceleration limits.
    if(0.1 < brake_scale)
    {
        // Set the brake limiter's acceleration limits based on the brake scale 
        // and defined brake deceleration, allowing for smooth deceleration 
        // when braking is applied.
        brake_limiter->SetMaxAcceleration( brake_scale * this->brake_deceleration);
        brake_limiter->SetMinAcceleration(-brake_scale * this->brake_deceleration);

        // When braking, we want to decelerate the vehicle smoothly to a stop,
        // so we set the target velocity to zero and let the brake limiter handle the deceleration.
        velocity = 0.0;

        // Use the brake limiter to compute the new velocity command, 
        // ensuring that the vehicle decelerates smoothly
        this->brake_limiter->Limit(
            velocity, 
            this->last_velocity_command0,
            this->last_velocity_command1, 
            _info.dt
        );
    }
    else
    {
        // When not braking, we want to apply the velocity command with acceleration limits,
        // so we use the velocity limiter to compute the new velocity command.
        this->velocity_limiter->Limit(
            velocity, 
            this->last_velocity_command0, 
            this->last_velocity_command1, 
            _info.dt
        );
    }

    // Apply the steering limiter to compute the new steering command,
    // ensuring that the steering changes smoothly and respects acceleration limits.
    this->steering_limiter->Limit(
        steering, 
        this->last_steering_command0, 
        this->last_steering_command1, 
        _info.dt
    );

    // Update the history of velocity and steering commands for use in the limiters.
    this->last_velocity_command1 = this->last_velocity_command0;
    this->last_velocity_command0 = velocity;
    this->last_steering_command1 = this->last_steering_command0;
    this->last_steering_command0 = steering;

    // Apply steering limits to ensure the steering angle 
    // does not exceed the maximum allowed value.
    if(this->steer_limit < std::abs(steering))
    {
        steering = (steering / std::abs(steering)) * this->steer_limit;
    }

    // Compute the tangent of the steering angle for use in the Ackermann steering calculations.
    const double tan_steer = std::tan(steering);

    const double left_target_steer = std::atan2(
        tan_steer, 1.0 - this->kingpin_width / 2.0 / this->wheel_base * tan_steer
    );

    const double right_target_steer = std::atan2(
        tan_steer, 1.0 + this->kingpin_width / 2.0 / this->wheel_base * tan_steer
    );

    // Retrieve the current joint positions for the left and right steering joints 
    // to compute the steering errors.
    const auto left_steer_position  = 
    _ecm.Component<components::JointPosition>(this->joint_entities[LEFT_STEER]);

    const auto right_steer_position = 
    _ecm.Component<components::JointPosition>(this->joint_entities[RIGHT_STEER]);

    if(!left_steer_position || !right_steer_position)
    {
        return;
    }

    // Compute the steering errors for the left and right steering joints,
    // and apply a proportional control law to compute the joint velocity commands for steering.
    const double left_steer_error  = left_target_steer  - left_steer_position->Data()[0];
    const double right_steer_error = right_target_steer - right_steer_position->Data()[0];

    this->joint_velocities[LEFT_STEER]  = this->steer_p_gain * left_steer_error;
    this->joint_velocities[RIGHT_STEER] = this->steer_p_gain * right_steer_error;

    // Compute the wheel velocities for each wheel based on the target velocity and steering angle,
    // using Ackermann steering geometry to ensure that the wheels turn at appropriate speeds
    // to achieve the desired steering angle while maintaining the target speed.
    this->joint_velocities[FRONT_LEFT] = velocity * 
        (1.0 - (this->front_wheel_tread * tan_steer) / (2.0 * this->wheel_base)) / 
        this->front_wheel_radius;

    this->joint_velocities[FRONT_RIGHT] = velocity * 
        (1.0 + (this->front_wheel_tread * tan_steer) / (2.0 * this->wheel_base)) / 
        this->front_wheel_radius;

    this->joint_velocities[REAR_LEFT] = velocity * 
        (1.0 - (this->rear_wheel_tread * tan_steer) / (2.0 * this->wheel_base)) / 
        this->rear_wheel_radius;

    this->joint_velocities[REAR_RIGHT] = velocity * 
        (1.0 + (this->rear_wheel_tread * tan_steer) / (2.0 * this->wheel_base)) / 
        this->rear_wheel_radius;
}

GZ_ADD_PLUGIN(
    ERP42Control,
    System,
    ERP42Control::ISystemConfigure,
    ERP42Control::ISystemPreUpdate,
    ERP42Control::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    ERP42Control,
    "gz::sim::systems::ERP42Control"
)