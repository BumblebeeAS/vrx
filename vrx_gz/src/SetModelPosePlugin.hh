/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
 */

#ifndef VRX_SET_MODEL_POSE_PLUGIN_HH_
#define VRX_SET_MODEL_POSE_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <memory>
#include <sdf/sdf.hh>

namespace vrx {

/// \brief Plugin to set the pose of a model using a joint position controller.
/// The plugin allows the pose of a model to be set dynamically via ROS
/// messages. The plugin accepts the following SDF parameters:
/// * Required parameters:
/// <model_name> - The name of the model to control.
/// <joint_name> - The name of the joint used to control the model's position.
/// <topic> - The ROS topic to receive pose commands.
///
/// Here's an example:
/// <plugin name="set_model_pose_plugin" filename="libset_model_pose_plugin.so">
///   <model_name>my_model</model_name>
///   <joint_name>my_joint</joint_name>
///   <topic>/my_model/set_pose</topic>
/// </plugin>
class SetModelPosePlugin : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate {
  // \brief Constructor.
public:
  SetModelPosePlugin();

  /// \brief Destructor.
public:
  ~SetModelPosePlugin() override = default;

  // Documentation inherited.
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Documentation inherited.
public:
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Private data pointer.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};

} // namespace vrx
#endif
