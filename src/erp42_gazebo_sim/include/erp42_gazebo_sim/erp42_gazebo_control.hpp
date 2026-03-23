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
 * @file    erp42_gazebo_control.hpp
 * @brief   ERP42 gazebo-sim control plugin
 * @author  Minkyu Kil
 * @date    2026-03-18
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_GAZEBO_SIM__ERP42_GAZEBO_CONTROL_HPP_
#define ERP42_GAZEBO_SIM__ERP42_GAZEBO_CONTROL_HPP_

#include "gz/sim/System.hh"
#include <memory>

namespace gz::sim
{
    inline namespace GZ_SIM_VERSION_NAMESPACE
    {
        namespace systems
        {
            /** @brief Private implementation for ERP42GazeboControl */
            class ERP42GazeboControlPrivate;

            /**
             * @brief ERP42GazeboControl is a Gazebo simulation plugin that controls the ERP42 model.
             * @details 
             *  - Subscribes to control commands
             *  - Manages the vehicle's state based on mode commands
             *  - Publishes feedback
             *  - Optionally publishes odometry information
             *  - Implements a simple proportional controller for steering and velocity control
             *  - Handles emergency stop and manual mode by overriding control commands
             */
            class ERP42GazeboControl : 
                public System, 
                public ISystemConfigure, 
                public ISystemPreUpdate, 
                public ISystemPostUpdate
            {
            public:

                /**
                 * @brief Constructor for ERP42GazeboControl plugin
                 * @details Initializes internal state and prepares for configuration
                 */
                ERP42GazeboControl();

                /** @brief Destructor for ERP42GazeboControl plugin */
                ~ERP42GazeboControl() override;

                /**
                 * @brief Configure plugin based on SDF parameters and initializes ROS2 interfaces
                 * @param[in] _entity The entity associated with the plugin (the ERP42 model)
                 * @param[in] _sdf The SDF element containing plugin parameters
                 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
                 * @param[in] _eventMgr The EventManager for subscribing to simulation events
                 * @details Parses SDF parameters, sets up ROS2 publishers/subscribers/services, 
                 * and initializes internal state
                 */
                void Configure(
                    const Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    EntityComponentManager &_ecm,
                    EventManager &_eventMgr
                ) override;

                /**
                 * @brief PreUpdate is called before the simulation update step
                 * @param[in] _info UpdateInfo containing timing and state information
                 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
                 * @details Can be used to perform any necessary preparations 
                 * before the simulation step, but in this plugin it is not used and is left empty
                 */
                void PreUpdate(
                    const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm
                ) override;

                /**
                 * @brief PostUpdate is called after the simulation update step
                 * @param[in] _info UpdateInfo containing timing and state information
                 * @param[in] _ecm The EntityComponentManager for interacting with the simulation
                 * @details Can be used to perform any necessary actions after the simulation step, 
                 * but in this plugin it is not used and is left empty
                 */
                void PostUpdate(
                    const UpdateInfo &_info,
                    const EntityComponentManager &_ecm
                ) override;

            private:

                /**
                 * @brief Pointer to the private implementation of the ERP42GazeboControl plugin
                 * @details Encapsulates all internal data and implementation details 
                 * to keep the public interface clean
                 */
                std::unique_ptr<ERP42GazeboControlPrivate> dataPtr;

            }; // class ERP42GazeboControl

        } // namespace systems

    } // inline namespace GZ_SIM_VERSION_NAMESPACE

} // namespace gz::sim

#endif // ERP42_GAZEBO_SIM__ERP42_GAZEBO_CONTROL_HPP_