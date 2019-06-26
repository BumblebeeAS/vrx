/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_
#define USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_

#include <map>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include "usv_gazebo_plugins/shape_volume.hh"

#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"

namespace gazebo
{
  namespace buoyancy {
    /// \brief A class for storing buoyancy object properties
    class BuoyancyObject {
      public:

      /// \brief Default constructor
      BuoyancyObject();

      /// \brief Default move constructor
      BuoyancyObject(BuoyancyObject&& obj) noexcept; // NOLINT

      /// \brief No copy constructor
      BuoyancyObject(BuoyancyObject& obj) = delete;

      /// \brief Loads buoyancy object from SDF
      void load(const physics::ModelPtr model, const sdf::ElementPtr elem);

      /// \brief Display string for buoyancy object
      std::string disp();

      /// \brief Associated link ID
      int linkId;

      /// \brief Associated link name
      std::string linkName;

      /// \brief Pose of buoyancy relative to link
      ignition::math::Pose3d pose;

      /// \brief Object mass (from inertial elem)
      double mass;

      /// \brief Buoyancy object's shape properties
      ::buoyancy::ShapeVolumePtr shape;
    };
  }  // end of buoyancy namespace

  /// \brief This plugin simulates buoyancy of an object in fluid.
  ///   <fluid_density>: Sets the density of the fluid that surrounds the
  ///                    buoyant object [kg/m^3].
  ///                    This parameter is optional (default value 997 kg/m^3).
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    This parameter is optional (default value 0m).
  ///
  ///   <linear_drag>:   Linear drag coeffient.
  ///                    This parameter is optional.
  ///
  ///   <angular_drag>:  Angular drag coeffient.
  ///                    This parameter is optional.
  ///
  ///   <buoyancy>:      Describes the volume properties
  ///                    For example:
  ///
  ///                    <buoyancy name="buoyancy1">
  ///                      <link_name>link</link_name>
  ///                      <geometry>
  ///                        ...
  ///                      </geometry>
  ///                    </buoyancy>
  ///
  ///     <link>:        Name of associated link element
  ///
  ///     <geometry>:    Geometry element specifying buoyancy object's
  ///                    volume properties.
  ///                    Supported shapes: box, sphere, cylinder
  class BuoyancyPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: BuoyancyPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water at 15 Celsius.
    protected: double fluidDensity;

    /// \brief The height of the fluid/air interface [m]. Defaults to 0.
    protected: double fluidLevel;

    /// \brief Linear drag coefficient. Defaults to 0.
    protected: double linearDrag;

    /// \brief Angular drag coefficient. Defaults to 0.
    protected: double angularDrag;

    /// \brief List of buoyancy objects for model
    protected: std::vector<buoyancy::BuoyancyObject> buoyancyObjects;

    /// \brief Map of <link ID, link pointer>
    protected: std::map<int, gazebo::physics::LinkPtr> linkMap;

    /// \brief Pointer to base model
    protected: physics::ModelPtr model;

    /// \brief Pointer to the Gazebo world
    /// Retrieved when the model is loaded.
    protected: physics::WorldPtr world;

    /// \brief The name of the wave model
    protected: std::string waveModel;

    /// \brief Vector of water height at each link from previous timestep
    protected: std::vector<double> buoyancyHeights;

    /// \brief Previous update time
    protected: double lastSimTime;

    /// \brief The wave parameters.
    protected: std::shared_ptr<const asv::WaveParameters> waveParams;
  };
}

#endif
