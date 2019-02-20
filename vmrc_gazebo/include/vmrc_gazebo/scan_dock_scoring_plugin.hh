/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef VMRC_GAZEBO_SCAN_DOCK_SCORING_PLUGIN_HH_
#define VMRC_GAZEBO_SCAN_DOCK_SCORING_PLUGIN_HH_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include "vmrc_gazebo/ColorSequence.h"
#include "vmrc_gazebo/scoring_plugin.hh"

/// \brief A plugin for computing the score of the scan and dock task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin requires the following SDF parameters:
///
/// <color_1>: Expected first color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_2>: Expected second color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_3>: Expected third color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <robot_namespace>: The ROS namespace for this plugin.
/// <color_sequence_service>: The ROS topic used to send the color pattern.
///
/// Here's an example:
/// <plugin name="scan_dock_scoring_plugin"
///         filename="libscan_code_scoring_plugin.so">
/// </plugin>
class ScanDockScoringPlugin : public ScoringPlugin
{
  typedef boost::weak_ptr<gazebo::physics::Entity> EntityWeakPtr;

  // Constructor.
  public: ScanDockScoringPlugin();

  // Documentation inherited.
  private: void Load(gazebo::physics::WorldPtr _world,
                     sdf::ElementPtr _sdf);

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  // Documentation inherited.
  private: void OnRunning() override;

  /// \brief Callback triggered when the vehicle enters or exits the activation
  /// zone.
  /// \param[in] _msg The current state (0: exiting, 1: entering).
  private: void OnActivationZoneBay1(ConstIntPtr &_msg);

  /// \brief Callback triggered when the vehicle enters or exits the activation
  /// zone.
  /// \param[in] _msg The current state (0: exiting, 1: entering).
  private: void OnActivationZoneBay2(ConstIntPtr &_msg);

  /// \brief Callback for change pattern service, calls other changePattern
  /// internally.
  /// \param[in] _req Not used.
  /// \param[out] _res The Response containing a message with the new pattern.
  /// \return True when the operation succeed or false otherwise.
  private: bool OnColorSequence(
  	ros::ServiceEvent<vmrc_gazebo::ColorSequence::Request,
  	  vmrc_gazebo::ColorSequence::Response> &_event);

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief The expected color sequence.
  private: std::vector<std::string> expectedSequence;

  /// \brief Service to generate and display a new color sequence.
  private: ros::ServiceServer colorSequenceServer;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic where the color sequence should be sent.
  private: std::string colorSequenceService = "/scan_dock/color_sequence";

  /// \brief Gazebo topic used to receive notifications about the bay #1.
  private: std::string bay1Topic;

  /// \brief Gazebo topic used to receive notifications about the bay #2.
  private: std::string bay2Topic;

  /// \brief Gazebo topic associated to the correct bay to dock.
  private: std::string correctBayTopic;

  /// \brief Whether the color sequence has been received or not.
  private: bool colorSequenceReceived = false;

  /// \brief Create a node for communication.
  private: gazebo::transport::NodePtr node;

  /// \brief Subscriber to receive notifications from the contain plugin.
  private: gazebo::transport::SubscriberPtr containSub1;

  /// \brief Subscriber to receive notifications from the contain plugin.
  private: gazebo::transport::SubscriberPtr containSub2;

  /// \brief Timer used to calculate the elapsed time docked in bay1.
  private: gazebo::common::Timer timer1;

  /// \brief Timer used to calculate the elapsed time docked in bay2.
  private: gazebo::common::Timer timer2;

  /// \brief Protect from race conditions.
  private: std::mutex mutex;
};

#endif
