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
 * @file    erp42_gazebo_control.cpp
 * @brief   ERP42 gazebo-sim control plugin
 * @author  Minkyu Kil
 * @date    2026-03-18
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_gazebo_sim/erp42_gazebo_control.hpp"
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

/** @brief Wheel joint indices */
enum WheelJoint : std::uint8_t
{
    // Front left steering
    LEFT_STEERING = 0,

    // Front right steering
    RIGHT_STEERING = 1,

    // Front left wheel
    FRONT_LEFT = 2,

    // Front right wheel
    FRONT_RIGHT = 3,

    // Rear left wheel
    REAR_LEFT = 4,

    // Rear right wheel
    REAR_RIGHT = 5,

    // Number of wheel joints
    JOINT_NUM

}; // enum WheelJoint

/** @brief Private implementation for ERP42GazeboControl */
class gz::sim::systems::ERP42GazeboControlPrivate
{
public:

    /**
     * @brief Callback function for control command subscription
     * @param[in] msg The received control command message
     * @details Updates target speed, steering, and brake based on the received command,
     * while respecting emergency stop and manual mode conditions. 
     * Uses mutex to protect against data races.
     */
    void OnControlCommand(const erp42_msgs::msg::ControlCommand::ConstSharedPtr &msg);

    /**
     * @brief Callback function for mode command service
     * @param[in] request The received mode command request containing emergency stop, 
     * manual mode, and gear information
     * @param[in] response The response to be sent back to the service caller, indicating success
     * @details Updates emergency stop, manual mode, and gear state based on the received request.
     * Uses mutex to protect against data races.
     */
    void OnModeCommand(
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
    void UpdateOdometry(const UpdateInfo &_info, const EntityComponentManager &_ecm);

    /**
     * @brief Updates feedback information and publishes it at a fixed rate
     * @param[in] _info UpdateInfo containing timing and state information
     * @param[in] _ecm The EntityComponentManager for interacting with the simulation
     * @details Retrieves the current joint positions and velocities, computes feedback information
     * such as steering angle and speed, and publishes it at a fixed rate defined by feedback_period.
     */
    void UpdateFeedback(const UpdateInfo &_info, const EntityComponentManager &_ecm);

    /**
     * @brief Updates control commands and applies them to the vehicle
     * @param[in] _info UpdateInfo containing timing and state information
     * @param[in] _ecm The EntityComponentManager for interacting with the simulation
     * @details Retrieves the current control commands, applies speed and steering limiters,
     * and updates the vehicle's joint velocities accordingly.
     */
    void UpdateControl(const UpdateInfo &_info, const EntityComponentManager &_ecm);

    // Mutex to protect data race from callback function
    std::mutex mutex;

    // Gazebo sim model
    Model model {kNullEntity};

    // Gazebo sim model canonical link
    Link canonicalLink {kNullEntity};

    // Angular velocity limiter : used to compute steering velocity command
    std::unique_ptr<math::SpeedLimiter> angularVelocityLimiter;

    // Velocity limter : used to compute driving velocity command
    std::unique_ptr<math::SpeedLimiter> linearVelocityLimiter;

    // Brake limter : used to compute braking velocity command
    std::unique_ptr<math::SpeedLimiter> brakeLimiter;

    // Ioniq5 wheel joint entities
    std::array<Entity, JOINT_NUM> joints{};

    // Ioniq5 wheel joint names
    std::array<std::string, JOINT_NUM> jointNames{};

    // Ioniq5 wheel joint velocity commands
    std::array<double, JOINT_NUM> jointVelocities{};

    // Wheel base : used to compute steering angle, simple differential gear 
    double wheelBase{1.040};

    // Kingpin width : used to compute steering angle
    double kingpinWidth{0.985};

    // Front wheel tread : used to compute simple differential gear
    double frontWheelTread{0.985};

    // Rear wheel tread : used to compute simple differential gear
    double rearWheelTread{0.985};

    // Front wheel radius : used to compute continuous joint velocity
    double frontWheelRadius{0.270};

    // Rear wheel radius : used to compute continuous joint velocity
    double rearWheelRadius{0.270};

    // P gain of the steering : used to compute steering velocity command
    double steeringPGain{10.0};

    // Max steering angle : used to clamp target steering
    double steeringLimit{25.0 * M_PI / 180.0};

    // Max vehicle velocity : used to clamp target velocity
    double velocityLimit{7.0};

    // Max vehicle acceleration : used to velocity limiter
    double accelerationLimit{2.5};

    // Max brake deceleration : used to brake limiter
    double brakeDeceleration{10.0};

    // Emergency stop mode : ignore all control commands
    bool emergencyStop {true};

    // Manual mode : ignore all control commands
    bool manualMode{true};

    // Gear mode : Override sign of the velocity
    std::uint8_t gear{1};

    // Target velocity : m/s
    double targetVelocity{0.0};

    // Target steering angle : rad
    double targetSteering{0.0};

    // Target brake ratio : 0 - 150
    std::uint8_t targetBrake{150};

    // Velcoity command history
    double lastLinearVelocityCommand0{0.0};
    double lastLinearVelocityCommand1{0.0};

    // Steering command history
    double lastAngularVelocityCommand0{0.0};
    double lastAngularVelocityCommand1{0.0};

    // Encoder angle history
    double prevEncoderAngle{0.0};

    // Encoder count : computed by CPR
    std::int32_t encoderCount{0};

    // Heartbeat : used to check aliveness
    std::uint8_t heartbeat{0};

    // Odometry topic and transform publication flag
    bool publishOdometry{true};

    // Odometry topic name
    std::string odometryTopic{"/odom"};

    // Odometry topic and transform frame ID
    std::string odometryFrame{"odom"};

    // Odometry topic and transform child frame ID
    std::string childFrame{"erp42"};

    // Odometry publication period
    std::chrono::steady_clock::duration odometryPeriod;

    // Last odometry publication time : used to limit odometry publication
    std::chrono::steady_clock::duration lastOdometryPublicationTime;

    // Feedback publication period
    std::chrono::steady_clock::duration feedbackPeriod {std::chrono::milliseconds(20)};

    // Last feedback publication time : used to limit feedback publication
    std::chrono::steady_clock::duration lastFeedbackPublicationTime;

    // ROS2 node to not use gazebo transport
    rclcpp::Node::SharedPtr node;

    // ROS2 node executor
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;

    // Context flag
    bool ownsContext {false};

    // Transform broadcaster for odometry frame
    std::shared_ptr<tf2_ros::TransformBroadcaster> odometryTransformBroadcaster;

    // Odometry topic publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

    // Feedback publisher
    rclcpp::Publisher<erp42_msgs::msg::Feedback>::SharedPtr feedbackPublisher;

    // Control command publisher
    rclcpp::Subscription<erp42_msgs::msg::ControlCommand>::SharedPtr controlCommandSubscriber;

    // Mode command service server
    rclcpp::Service<erp42_msgs::srv::ModeCommand>::SharedPtr modeCommandService;

}; // class gz::sim::systems::ERP42GazeboControlPrivate

/**
 * @brief Constructor for ERP42GazeboControl plugin
 * @details Initializes internal state and prepares for configuration
 */
ERP42GazeboControl::ERP42GazeboControl() 
  : dataPtr(std::make_unique<ERP42GazeboControlPrivate>())
{
}

ERP42GazeboControl::~ERP42GazeboControl()
{
    // Private class is removed already
    if(!this->dataPtr)
    {
        return;
    }

    // Reset ROS2 entities
    if(this->dataPtr->executor && this->dataPtr->node)
    {
        // Check context and reset
        rclcpp::Context::SharedPtr context =
        this->dataPtr->node->get_node_base_interface()->get_context();

        if(context && context->is_valid())
        {
            this->dataPtr->executor->remove_node(this->dataPtr->node);
        }

        // Reset ROS2 communication entities
        this->dataPtr->odometryTransformBroadcaster.reset();
        this->dataPtr->odometryPublisher.reset();
        this->dataPtr->feedbackPublisher.reset();
        this->dataPtr->controlCommandSubscriber.reset();
        this->dataPtr->modeCommandService.reset();

        // Reset ROS2 node and executor 
        this->dataPtr->node.reset();
        this->dataPtr->executor.reset();
    }
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
void ERP42GazeboControl::Configure(
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
        gzerr 
        << "ERP42GazeboControl should be attached to a model entity. " 
        << "Failed to initialize." 
        << std::endl;
        return;
    }

    // Find the canonical link of the model
    std::vector<Entity> links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(),
        components::CanonicalLink()
    );
    if(!links.empty())
    {
        this->dataPtr->canonicalLink = Link(links[0]);
    }

    // Parse joint names from SDF, using default values if not specified
    this->dataPtr->jointNames[LEFT_STEERING] = _sdf->Get<std::string>(
        "left_steering_joint", "left_steering_joint"
    ).first;

    this->dataPtr->jointNames[RIGHT_STEERING] = _sdf->Get<std::string>(
        "right_steering_joint", "right_steering_joint"
    ).first;

    this->dataPtr->jointNames[FRONT_LEFT] = _sdf->Get<std::string>(
        "front_left_wheel_joint", "front_left_wheel_joint"
    ).first;

    this->dataPtr->jointNames[FRONT_RIGHT] = _sdf->Get<std::string>(
        "front_right_wheel_joint", "front_right_wheel_joint"
    ).first;

    this->dataPtr->jointNames[REAR_LEFT] = _sdf->Get<std::string>(
        "rear_left_wheel_joint", "rear_left_wheel_joint"
    ).first;

    this->dataPtr->jointNames[REAR_RIGHT] = _sdf->Get<std::string>(
        "rear_right_wheel_joint", "rear_right_wheel_joint"
    ).first;

    // Parse vehicle geometry parameters from SDF, using default values if not specified
    this->dataPtr->wheelBase = _sdf->Get<double>(
        "wheel_base", this->dataPtr->wheelBase
    ).first;

    this->dataPtr->kingpinWidth = _sdf->Get<double>(
        "kingpin_width", this->dataPtr->kingpinWidth
    ).first;

    this->dataPtr->frontWheelTread = _sdf->Get<double>(
        "front_wheel_tread", this->dataPtr->frontWheelTread
    ).first;

    this->dataPtr->rearWheelTread = _sdf->Get<double>(
        "rear_wheel_tread", this->dataPtr->rearWheelTread
    ).first;

    this->dataPtr->frontWheelRadius = _sdf->Get<double>(
        "front_wheel_radius", this->dataPtr->frontWheelRadius
    ).first;

    this->dataPtr->rearWheelRadius = _sdf->Get<double>(
        "rear_wheel_radius", this->dataPtr->rearWheelRadius
    ).first;

    // Parse control limits from SDF, using default values if not specified
    this->dataPtr->steeringPGain = _sdf->Get<double>(
        "steering_p_gain", this->dataPtr->steeringPGain
    ).first;

    this->dataPtr->steeringLimit = _sdf->Get<double>(
        "steering_limit", this->dataPtr->steeringLimit
    ).first;

    this->dataPtr->velocityLimit = _sdf->Get<double>(
        "velocity_limit", this->dataPtr->velocityLimit
    ).first;

    this->dataPtr->accelerationLimit = _sdf->Get<double>(
        "acceleration_limit", this->dataPtr->accelerationLimit
    ).first;

    this->dataPtr->brakeDeceleration = _sdf->Get<double>(
        "brake_deceleration", this->dataPtr->brakeDeceleration
    ).first;

    // Initialize steering limiters
    this->dataPtr->angularVelocityLimiter = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->angularVelocityLimiter->SetMaxVelocity( 1.0 * this->dataPtr->velocityLimit);
    this->dataPtr->angularVelocityLimiter->SetMinVelocity(-1.0 * this->dataPtr->velocityLimit);

    // Initialize steering limiter
    this->dataPtr->linearVelocityLimiter = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->linearVelocityLimiter->SetMaxVelocity( 1.0 * this->dataPtr->velocityLimit);
    this->dataPtr->linearVelocityLimiter->SetMinVelocity(-1.0 * this->dataPtr->velocityLimit);
    this->dataPtr->linearVelocityLimiter->SetMaxAcceleration( 1.0 * this->dataPtr->accelerationLimit);
    this->dataPtr->linearVelocityLimiter->SetMinAcceleration(-1.0 * this->dataPtr->accelerationLimit);

    // Initialize brake limiter : used to UpdateControl, limiter set by braking ratio
    this->dataPtr->brakeLimiter = std::make_unique<math::SpeedLimiter>();

    // Parse odometry publication options from SDF, using default values if not specified
    this->dataPtr->publishOdometry = _sdf->Get<bool>(
        "publish_odometry", this->dataPtr->publishOdometry
    ).first;

    this->dataPtr->odometryTopic = _sdf->Get<std::string>(
        "odometry_topic", this->dataPtr->odometryTopic
    ).first;

    this->dataPtr->odometryFrame = _sdf->Get<std::string>(
        "odometry_frame", this->dataPtr->odometryFrame
    ).first;

    this->dataPtr->childFrame = _sdf->Get<std::string>(
        "child_frame", this->dataPtr->childFrame
    ).first;

    const double odometryFrequency = _sdf->Get<double>("odometry_frequency", 100).first;
    if(0 < odometryFrequency)
    {
        this->dataPtr->odometryPeriod = 
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / odometryFrequency)
        );
    }

    // Initialize ROS2 node and executors
    if(!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
        this->dataPtr->ownsContext = true;
    }

    rclcpp::NodeOptions nodeOptions;
    nodeOptions.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    this->dataPtr->node = std::make_shared<rclcpp::Node>("erp42_gazebo_control", nodeOptions);

    this->dataPtr->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->dataPtr->executor->add_node(this->dataPtr->node);
    
    // Setup ROS2 communication entites
    rclcpp::QoS erp42GazeboControlQoS(rclcpp::KeepLast(1));
    erp42GazeboControlQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);
    erp42GazeboControlQoS.durability(rclcpp::DurabilityPolicy::Volatile);

    if(this->dataPtr->publishOdometry)
    {
        // Odometry transform broadcaster
        this->dataPtr->odometryTransformBroadcaster = 
        std::make_shared<tf2_ros::TransformBroadcaster>(this->dataPtr->node);

        // Odometry topic publisher
        this->dataPtr->odometryPublisher = 
        this->dataPtr->node->create_publisher<nav_msgs::msg::Odometry>(
            this->dataPtr->odometryTopic, erp42GazeboControlQoS
        );
    }

    // Feedback publisher
    this->dataPtr->feedbackPublisher = 
    this->dataPtr->node->create_publisher<erp42_msgs::msg::Feedback>(
        "/erp42/feedback", erp42GazeboControlQoS
    );

    // Control command subscription
    this->dataPtr->controlCommandSubscriber = 
    this->dataPtr->node->create_subscription<erp42_msgs::msg::ControlCommand>(
        "/erp42/control_command", 
        erp42GazeboControlQoS,
        [this](const erp42_msgs::msg::ControlCommand::ConstSharedPtr &msg)
        {
            this->dataPtr->OnControlCommand(msg);
        }
    );

    // Mode command service server
    this->dataPtr->modeCommandService = 
    this->dataPtr->node->create_service<erp42_msgs::srv::ModeCommand>(
        "/erp42/mode_command",
        [this](
            const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
            erp42_msgs::srv::ModeCommand::Response::SharedPtr response
        )
        {
            this->dataPtr->OnModeCommand(request, response);
        }
    );

    // Serial bridge default port path
    this->dataPtr->node->declare_parameter<std::string>("port_path", "/dev/ttyUSB0");

    // Serial bridge default baud rate
    this->dataPtr->node->declare_parameter<int>("baud_rate", 115200);

    // Velocity limit m/s
    this->dataPtr->node->declare_parameter<double>(
        "max_speed_mps", this->dataPtr->velocityLimit
    );

    // Steering limit
    this->dataPtr->node->declare_parameter<double>(
        "max_steering_deg", this->dataPtr->steeringLimit * 180.0 / M_PI
    );

    // Steering offest
    this->dataPtr->node->declare_parameter<double>(
        "steering_offset_deg", 0.0
    );
}

void ERP42GazeboControl::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42GazeboControl::PreUpdate");

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

    // Check if joint entities have been found, 
    // and if not, attempt to find them again.
    const bool nullEntityFound = std::any_of(
        this->dataPtr->joints.begin(),
        this->dataPtr->joints.end(),
        [](Entity entity){ return entity == kNullEntity; }
    );

    // If any joint entity is still null, attempt to find them again. 
    // This can happen if the plugin is loaded before the model is fully initialized.
    if(nullEntityFound)
    {
        // Get the model name for logging purposes
        const auto modelName = this->dataPtr->model.Name(_ecm);

        // Use a static set to track which models have already been warned about missing joints,
        // to avoid spamming warnings for the same model on every update.
        bool warned = false;
        static std::set<std::string> warnedModels;

        // Try to find joint entities again
        for(std::uint8_t idx = 0; idx < JOINT_NUM; idx++)
        {
            const std::string jointName = this->dataPtr->jointNames[idx];
            const Entity jointEntity = this->dataPtr->model.JointByName(_ecm, jointName);

            // If the joint entity is found, store it in the joint_entities array.
            if(jointEntity != kNullEntity)
            {
                this->dataPtr->joints[idx] = jointEntity;
            }
            else
            {
                gzwarn 
                << "Failed to find joint [" << jointName << "] " << "for model [" << modelName << "]" 
                << std::endl;
                warned = true;
            }
        }

        // If we failed to find any joints,
        // and we haven't already warned about this model, log a warning.
        if(warned)
        {
            warnedModels.insert(modelName);
            return;
        }

        // If we successfully found all joints, and we had previously warned about this model, 
        // log a message and remove it from the warned set.
        if(warnedModels.find(modelName) != warnedModels.end())
        {
            gzmsg 
            << "Found joints for model [" << modelName << "], " << "plugin will start working." 
            << std::endl;
            warnedModels.erase(modelName);
        }
    }

    // If the simulation is paused, 
    // do not update joint commands or create necessary components.
    if(_info.paused)
    {
        return;
    }

    // Set joint velocity commands based on the current target velocities.
    for(std::uint8_t idx = 0; idx < JOINT_NUM; idx++)
    {
        _ecm.SetComponentData<components::JointVelocityCmd>(
            this->dataPtr->joints[idx],
            {this->dataPtr->jointVelocities[idx]}
        );
    }

    // Ensure that necessary components for joints and canonical link exist,
    // creating them if they are missing. This is important for the plugin to function correctly,
    // especially if the plugin is loaded before the model is fully initialized.
    for(std::uint8_t idx = 0; idx < JOINT_NUM; idx++)
    {
        // Check if JointPosition component exists for the joint, 
        // and create it if it doesn't
        auto jointPosition = _ecm.Component<components::JointPosition>(this->dataPtr->joints[idx]);

        // If the JointPosition component is missing, create it. 
        // This is necessary for the plugin to function correctly,
        if(!jointPosition)
        {
            _ecm.CreateComponent(this->dataPtr->joints[idx], components::JointPosition());
        }

        // Check if JointVelocity component exists for the joint,
        // and create it if it doesn't
        auto jointVelocity = _ecm.Component<components::JointVelocity>(this->dataPtr->joints[idx]);

        // If the JointVelocity component is missing, create it.
        if(!jointVelocity)
        {
            _ecm.CreateComponent(this->dataPtr->joints[idx], components::JointVelocity());
        }
    }

    // Ensure that WorldPose, WorldLinearVelocity, and WorldAngularVelocity components exist 
    // for the canonical link, creating them if they are missing. 
    // This is important for the plugin to function correctly, 
    // especially if the plugin is loaded before the model is fully initialized.
    if(!this->dataPtr->publishOdometry)
    {
        return;
    }

    if(!_ecm.Component<components::WorldPose>(this->dataPtr->canonicalLink.Entity()))
    {
        _ecm.CreateComponent(
            this->dataPtr->canonicalLink.Entity(), 
            components::WorldPose()
        );
    }

    if(!_ecm.Component<components::WorldLinearVelocity>(this->dataPtr->canonicalLink.Entity()))
    {
        _ecm.CreateComponent(
            this->dataPtr->canonicalLink.Entity(), 
            components::WorldLinearVelocity()
        );
    }

    if(!_ecm.Component<components::WorldAngularVelocity>(this->dataPtr->canonicalLink.Entity()))
    {
        _ecm.CreateComponent(
            this->dataPtr->canonicalLink.Entity(), 
            components::WorldAngularVelocity()
        );
    }
}

/**
 * @brief Post-update function for ERP42GazeboControl plugin, called after each simulation update
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Handles ROS2 communication, updates odometry and feedback information, 
 * and applies control commands
 */
void ERP42GazeboControl::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42GazeboControl::PostUpdate");

    // If the simulation is paused, 
    // do not update odometry, feedback, or control.
    if(_info.paused)
    {
        return;
    }

    // Check if joint entities have been found, and if not, do not proceed with updates.
    const bool nullEntityFound = std::any_of(
        this->dataPtr->joints.begin(),
        this->dataPtr->joints.end(),
        [](Entity entity){ return entity == kNullEntity; }
    );

    // If any joint entity is still null, do not proceed with updates, 
    // as the plugin is not fully initialized.
    if(nullEntityFound)
    {
        return;
    }
    // Process ROS2 callbacks to handle incoming messages and service requests.
    if(this->dataPtr->executor)
    {
        rclcpp::Context::SharedPtr context =
        this->dataPtr->node->get_node_base_interface()->get_context();

        if(context && context->is_valid())
        {
            this->dataPtr->executor->spin_some();
        }
    }

    // Update odometry information and publish it if enabled.
    if(this->dataPtr->publishOdometry)
    {
        this->dataPtr->UpdateOdometry(_info, _ecm);
    }

    // Update feedback data
    this->dataPtr->UpdateFeedback(_info, _ecm);

    // Update wheel joint velocities
    this->dataPtr->UpdateControl(_info, _ecm);
}

/**
 * @brief Updates feedback information and publishes it at a fixed rate
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current joint positions and velocities, computes feedback information
 * such as steering angle and speed, and publishes it at a fixed rate defined by feedback_period.
 */
void ERP42GazeboControlPrivate::UpdateFeedback(
    const UpdateInfo &_info, 
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42GazeboControlPrivate::UpdateFeedback");

    // Check if it's time to publish the next feedback message based on the defined frequency.
    if((_info.simTime - this->lastFeedbackPublicationTime) < this->feedbackPeriod)
    {
        return;
    }

    // Update the last publication time to the current simulation time.
    this->lastFeedbackPublicationTime = _info.simTime;

    // Retrieve joint positions and velocities for steering and wheel joints.
    const auto leftSteeringPosition = 
    _ecm.Component<components::JointPosition>(this->joints[LEFT_STEERING]);

    const auto rightSteeringPosition = 
    _ecm.Component<components::JointPosition>(this->joints[RIGHT_STEERING]);

    // Wheel position : using actual encoder
    const auto frontLeftWheelPosition = 
    _ecm.Component<components::JointPosition>(this->joints[FRONT_LEFT]);

    // Wheel velocity : using actual encoder
    const auto frontLeftWheelVelocity = 
    _ecm.Component<components::JointVelocity>(this->joints[FRONT_LEFT]);

    if(
        !leftSteeringPosition   || leftSteeringPosition->Data().empty()   ||
        !rightSteeringPosition  || rightSteeringPosition->Data().empty()  ||
        !frontLeftWheelPosition || frontLeftWheelPosition->Data().empty() ||
        !frontLeftWheelVelocity || frontLeftWheelVelocity->Data().empty()
    )
    {
        return;
    }

    // Convert simulation time to ROS2 time for message timestamps.
    const rclcpp::Time stamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            _info.simTime
        ).count()
    );

    erp42_msgs::msg::Feedback feedback;
    feedback.header.stamp = stamp;
    feedback.header.frame_id = this->childFrame;

    // Set emergency stop
    feedback.emergency_stop = this->emergencyStop;

    // Set manual mode
    feedback.manual_mode = this->manualMode;
    
    // Set gear
    feedback.gear = this->gear;

    // Set steering (must be computed before speed for Ackermann correction)
    const double tanLeft  = std::tan(leftSteeringPosition->Data()[0]);
    const double tanRight = std::tan(rightSteeringPosition->Data()[0]);
    const double tanSum   = tanLeft + tanRight;
    feedback.steering = 
        (std::abs(tanSum) > 1e-6) ? std::atan(2.0 * tanLeft * tanRight / tanSum) : 0.0;

    // Set speed with Ackermann correction
    const double frontLeftFactor = 
        1.0 - (this->frontWheelTread * std::tan(feedback.steering)) / (2.0 * this->wheelBase);

    const double frontLeftWheelSpeed =
        frontLeftWheelVelocity->Data()[0] * this->frontWheelRadius;

    feedback.speed = 
        (std::abs(frontLeftFactor) > 0.01) ? 
        (frontLeftWheelSpeed / frontLeftFactor) : 
        frontLeftWheelSpeed;

    // Set brake
    feedback.brake = this->targetBrake;

    // Update encoder count
    const double encoderAngle = frontLeftWheelPosition->Data()[0];
    const double deltaEncoderAngle = std::remainder(
        encoderAngle - this->prevEncoderAngle, 2.0 * M_PI
    );

    // Compute encoder position difference
    if((2.0 * M_PI / erp42_util::ENCODER_CPR) < std::abs(deltaEncoderAngle))
    {
        // Compute encoder count
        this->encoderCount += static_cast<int>(
            deltaEncoderAngle / (2.0 * M_PI) * erp42_util::ENCODER_CPR
        );

        // Update history
        this->prevEncoderAngle = encoderAngle;
    }

    // Set encoder count
    feedback.encoder_count = this->encoderCount;

    // Set heartbeat
    feedback.heartbeat = this->heartbeat++;

    // Publish feedback message
    this->feedbackPublisher->publish(feedback);
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
void ERP42GazeboControlPrivate::UpdateControl(
    const UpdateInfo &_info, 
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42GazeboControlPrivate::UpdateControl");

    // Retrieve the desired gear mode states and control commands,
    // while holding the mutex to ensure thread safety.
    double linearVelocity, steering, brakeRatio;
    {
        // Lock the mutex to protect against data
        std::lock_guard<std::mutex> lock(this->mutex);

        // Read target control commands
        linearVelocity = this->targetVelocity;
        steering       = this->targetSteering;
        brakeRatio     = static_cast<double>(this->targetBrake) / 150.0;

        // Overrides control commands by safety mode commands
        if(this->emergencyStop || this->manualMode)
        {
            linearVelocity = 0.0;
            steering       = 0.0;
            brakeRatio     = 1.0;
        }

        // Overrides control commands
        switch(this->gear)
        {
        case 0: // GEAR_DRIVE
            linearVelocity = std::abs(linearVelocity);
            break;

        case 2: // GEAR_REVERSE
            linearVelocity = -1.0 * std::abs(linearVelocity);
            break;
            
        default: // GEAR_NEUTRAL or invalid gear
            linearVelocity = 0.0;
            brakeRatio     = 1.0;
        }
    }

    // Apply brake limiter if brake ratio is above threshold, 
    // otherwise apply normal speed limiter.
    if(0.1 < brakeRatio)
    {
        this->brakeLimiter->SetMaxAcceleration( 1.0 * brakeRatio * this->brakeDeceleration);
        this->brakeLimiter->SetMinAcceleration(-1.0 * brakeRatio * this->brakeDeceleration);

        linearVelocity = 0.0;

        this->brakeLimiter->Limit(
            linearVelocity, 
            this->lastLinearVelocityCommand0,
            this->lastLinearVelocityCommand1, 
            _info.dt
        );
    }
    else
    {
        this->linearVelocityLimiter->Limit(
            linearVelocity, 
            this->lastLinearVelocityCommand0,
            this->lastLinearVelocityCommand1, 
            _info.dt
        );
    }

    // Update velocity command history for the linear velocity limiter.
    this->lastLinearVelocityCommand1 = this->lastLinearVelocityCommand0;
    this->lastLinearVelocityCommand0 = linearVelocity;

    // Compute the desired angular velocity based on the linear velocity and steering angle,
    // and apply the angular velocity limiter to ensure it stays within limits.
    double angularVelocity = linearVelocity * std::tan(steering) / this->wheelBase;

    this->angularVelocityLimiter->Limit(
        angularVelocity, 
        this->lastAngularVelocityCommand0,
        this->lastAngularVelocityCommand1,
        _info.dt
    );

    this->lastAngularVelocityCommand1 = this->lastAngularVelocityCommand0;
    this->lastAngularVelocityCommand0 = angularVelocity;

    // Compute the turning radius based on the linear and angular velocities,
    // and apply Ackermann steering geometry to compute the target steering angles.
    const double minTurningRadius = this->wheelBase / std::sin(this->steeringLimit);
    double turningRadius = linearVelocity / angularVelocity;
    
    // Apply steering limits to the turning radius to ensure 
    // it does not exceed the physical capabilities of the vehicle.
    if(0.0 < std::abs(linearVelocity))
    {
        if((0.0 <= turningRadius) && (turningRadius < minTurningRadius))
        {
            turningRadius = minTurningRadius;
        }
        if((turningRadius <= 0.0) && (-minTurningRadius < turningRadius))
        {
            turningRadius = -minTurningRadius;
        }
    }
    else if(0.001 <= std::abs(angularVelocity))
    {
        turningRadius = (angularVelocity / std::abs(angularVelocity)) * minTurningRadius;
    }
    if(std::abs(angularVelocity) < 0.001)
    {
        turningRadius = 1000000000.0;
    }

    // Compute the target steering angles for the left and right wheels 
    // using Ackermann steering geometry, based on the turning radius.
    const double leftSteeringTarget  = std::atan(
        this->wheelBase / (turningRadius - (this->kingpinWidth / 2.0))
    );
    const double rightSteeringTarget = std::atan(
        this->wheelBase / (turningRadius + (this->kingpinWidth / 2.0))
    );
    const double phi = std::atan(this->wheelBase / turningRadius);

    this->jointVelocities[FRONT_LEFT] = (
        linearVelocity * (1.0 - (this->frontWheelTread * std::tan(phi)) / (2.0 * this->wheelBase))
    ) / this->frontWheelRadius;

    this->jointVelocities[REAR_LEFT] = (
        linearVelocity * (1.0 - (this->rearWheelTread  * std::tan(phi)) / (2.0 * this->wheelBase))
    ) / this->rearWheelRadius;

    this->jointVelocities[FRONT_RIGHT] = (
        linearVelocity * (1.0 + (this->frontWheelTread * std::tan(phi)) / (2.0 * this->wheelBase))
    ) / this->frontWheelRadius;

    this->jointVelocities[REAR_RIGHT] = (
        linearVelocity * (1.0 + (this->rearWheelTread  * std::tan(phi)) / (2.0 * this->wheelBase))
    ) / this->rearWheelRadius;

    // Compute the steering errors for the left and right steering joints,
    // and update the joint velocities for the steering joints using a proportional controller.
    const auto leftSteeringPosition = _ecm.Component<components::JointPosition>(
        this->joints[LEFT_STEERING]
    );

    const auto rightSteeringPosition = _ecm.Component<components::JointPosition>(
        this->joints[RIGHT_STEERING]
    );

    if (
        !leftSteeringPosition  || leftSteeringPosition->Data().empty() ||
        !rightSteeringPosition || rightSteeringPosition->Data().empty()
    )
    {
        return;
    }

    const double leftSteeringError  = leftSteeringTarget  - leftSteeringPosition->Data()[0];
    const double rightSteeringError = rightSteeringTarget - rightSteeringPosition->Data()[0];

    this->jointVelocities[LEFT_STEERING]  = this->steeringPGain * leftSteeringError;
    this->jointVelocities[RIGHT_STEERING] = this->steeringPGain * rightSteeringError;
}

/**
 * @brief Updates odometry information and publishes it if the publish_odometry option is enabled
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current pose and velocity of the vehicle, constructs an odometry message,
 * and publishes it if the publish_odometry option is enabled. 
 * Also broadcasts the corresponding transform for the odometry frame.
 */
void ERP42GazeboControlPrivate::UpdateOdometry(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm
)
{
    GZ_PROFILE("ERP42GazeboControlPrivate::UpdateOdometry");

    // Check if it's time to publish the next odometry message 
    // based on the defined frequency.
    if((_info.simTime - this->lastOdometryPublicationTime) < this->odometryPeriod)
    {
        return;
    }

    // Update the last publication time to the current simulation time.
    this->lastOdometryPublicationTime = _info.simTime;

    // Retrieve the current pose, linear velocity, 
    // and angular velocity of the canonical link.
    auto pose = this->canonicalLink.WorldPose(_ecm);
    auto linearVelocity = this->canonicalLink.WorldLinearVelocity(_ecm);
    auto angularVelocity = this->canonicalLink.WorldAngularVelocity(_ecm);

    if(!pose || !linearVelocity || !angularVelocity)
    {
        return;
    }

    // Convert simulation time to ROS2 time for message timestamps.
    const rclcpp::Time stamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            _info.simTime
        ).count()
    );

    // Odometry header
    std_msgs::msg::Header header;
    header.frame_id = this->odometryFrame;
    header.stamp = stamp;

    // Construct the odometry message 
    // with the current pose and velocity information.
    nav_msgs::msg::Odometry odometry;
    odometry.header                  = header;
    odometry.child_frame_id          = this->childFrame;
    odometry.pose.pose.position.x    = pose->Pos().X();
    odometry.pose.pose.position.y    = pose->Pos().Y();
    odometry.pose.pose.position.z    = pose->Pos().Z();
    odometry.pose.pose.orientation.x = pose->Rot().X();
    odometry.pose.pose.orientation.y = pose->Rot().Y();
    odometry.pose.pose.orientation.z = pose->Rot().Z();
    odometry.pose.pose.orientation.w = pose->Rot().W();
    odometry.twist.twist.linear.x    = linearVelocity->X();
    odometry.twist.twist.linear.y    = linearVelocity->Y();
    odometry.twist.twist.linear.z    = linearVelocity->Z();
    odometry.twist.twist.angular.x   = angularVelocity->X();
    odometry.twist.twist.angular.y   = angularVelocity->Y();
    odometry.twist.twist.angular.z   = angularVelocity->Z();

    // Broadcast the transform corresponding 
    // to the odometry frame for use in ROS2 Transform.
    geometry_msgs::msg::TransformStamped transform;
    transform.header                  = header;
    transform.child_frame_id          = this->childFrame;
    transform.transform.translation.x = pose->Pos().X();
    transform.transform.translation.y = pose->Pos().Y();
    transform.transform.translation.z = pose->Pos().Z();
    transform.transform.rotation      = odometry.pose.pose.orientation;

    // Publish the odometry message and broadcast the transform.
    this->odometryPublisher->publish(odometry);
    this->odometryTransformBroadcaster->sendTransform(transform);
}

/**
 * @brief Updates control commands and applies them to the vehicle
 * @param[in] _info UpdateInfo containing timing and state information
 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
 * @details Retrieves the current control commands, applies speed and steering limiters,
 * and updates the vehicle's joint velocities accordingly.
 */
void ERP42GazeboControlPrivate::OnControlCommand(
    const erp42_msgs::msg::ControlCommand::ConstSharedPtr &msg
)
{
    // Lock the mutex to protect against data
    std::lock_guard<std::mutex> lock(this->mutex);

    // Update target speed, steering, and brake from the received control command message,
    // applying limits to ensure they are within the defined constraints.
    this->targetVelocity = std::clamp(msg->speed, 0.0, this->velocityLimit);

    // Steering command is clamped within the defined steering limits 
    // to prevent excessive steering angles.
    this->targetSteering = std::clamp(
        msg->steering, -1.0 * this->steeringLimit, 1.0 * this->steeringLimit
    );

    // Brake command is clamped between 0 and 150, where 150 represents full braking force.
    this->targetBrake = std::clamp<std::uint8_t>(msg->brake, 0, 150);
}

/**
 * @brief Service callback for mode commands, 
 * allowing external control over emergency stop, manual mode, and gear state
 */
void ERP42GazeboControlPrivate::OnModeCommand(
    const erp42_msgs::srv::ModeCommand::Request::SharedPtr request,
    erp42_msgs::srv::ModeCommand::Response::SharedPtr response
)
{
    // Lock the mutex to protect against concurrent access to mode state variables,
    // ensuring thread safety when processing mode command service requests.
    std::lock_guard<std::mutex> lock(this->mutex);

    // If emergency stop is activated, 
    // it will override manual mode and gear settings in the control update.
    this->emergencyStop = request->emergency_stop;

    // Update mode states based on the received service request.
    this->manualMode = request->manual_mode;

    // Update gear state based on the received service request.
    this->gear = request->gear;

    // Set the response success flag to true to indicate 
    // that the mode command was processed successfully.
    response->success = true;
}

GZ_ADD_PLUGIN(
    ERP42GazeboControl,
    System,
    ERP42GazeboControl::ISystemConfigure,
    ERP42GazeboControl::ISystemPreUpdate,
    ERP42GazeboControl::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    ERP42GazeboControl,
    "gz::sim::systems::ERP42GazeboControl"
)
