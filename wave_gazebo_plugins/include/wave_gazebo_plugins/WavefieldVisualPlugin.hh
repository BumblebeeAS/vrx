/*
 * Copyright (C) 2019  Rhys Mainwaring
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

/// \file WavefieldVisualPlugin.hh
/// \brief This file defines a Gazebo VisualPlugin used to render
/// a wave field and keep it synchronised with the model used in
/// the physics engine.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_VISUAL_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_VISUAL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>

#include <memory>

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  /// \internal
  /// \brief Class to hold private data for WavefieldModelPlugin.
  class WavefieldVisualPluginPrivate;

  /// \brief A Gazebo visual plugin to synchronise and control
  /// a vertex shader rendering Gerstner waves. It also renders reflections
  /// and refractions onto the water.
  ///
  /// # Usage
  ///
  /// Add the SDF for the plugin to the <visual> element of your wave model.
  ///
  /// The SDF parameters specifying the wave are all optional, and normal
  /// use will be overridden.
  ///
  /// If this visual is loaded as part of a wave model that also contains
  /// the plugin libWavefieldModelPlugin.so, then it will receive a response
  /// to its request for ~/wave_param and set the wave parameters to be
  /// consistent with the wave generator operating on the physics server.
  ///
  /// \code
  /// <plugin name="wavefield_visual" filename="libWavefieldVisualPlugin.so">
  ///   <shallowRefractRatio>0.1</shallowRefractRatio>
  ///   <envReflectRatio>0.5</envReflectRatio>
  ///   <static>false</static>
  ///   <wave>
  ///     <number>3</number>
  ///     <scale>1.5</scale>
  ///     <angle>0.4</angle>
  ///     <steepness>1.0</steepness>
  ///     <amplitude>0.4</amplitude>
  ///     <period>8.0</period>
  ///     <direction>1 1</direction>
  ///   </wave>
  /// </plugin>
  /// \endcode
  ///
  /// # Subscribed Topics
  ///
  /// 1. ~/reponse (gazebo::msgs::Response)
  ///
  /// 2. ~/wave (gazebo::msgs::Param_V)
  ///
  /// 3. ~/world_stats (gazebo::msgs::WorldStatistics)
  ///
  /// 4. /marker (ignition::msgs::Marker)
  ///
  /// # Published Topics
  ///
  /// 1. ~/request (gazebo::msgs::Request)
  ///
  /// # Parameters
  ///
  /// 1. <static> (bool, default: false)
  ///   Display a static wave field if set to true.
  ///
  /// 2. <shallowRefractRatio> (double, default: 0.2)
  ///   Ratio between shallow water color and refraction color to use
  ///   In [0, 1], where 0 is no refraction and 1 is maximum refraction
  ///
  /// 3. <envReflectRatio> (double, default: 0.2)
  ///    Ratio between environment color and reflection color to use
  ///    In [0, 1], where 0 is no reflection and 1 is maximum reflection
  ///
  /// 4. <number> (int, default: 1)
  ///   The number of component waves.
  ///
  /// 5. <scale> (double, default: 2.0)
  ///   The scale between the mean and largest / smallest component waves.
  ///
  /// 6. <angle> (double, default: 2*pi/10)
  ///   The angle between the mean wave direction and the
  ///   largest / smallest component waves.
  ///
  /// 7. <steepness> (double, default: 1.0)
  ///   A parameter in [0, 1] controlling the wave steepness
  ///   with 1 being steepest.
  ///
  /// 8. <amplitude> (double, default: 0.0)
  ///   The amplitude of the mean wave in [m].
  ///
  /// 9. <period> (double, default: 1.0)
  ///   The period of the mean wave in [s].
  ///
  /// 10. <phase> (double, default: 0.0)
  ///   The phase of the mean wave.
  ///
  /// 11. <direction> (Vector2D, default: (1 0))
  ///   A two component vector specifiying the direction of the mean wave.
  ///
  class GZ_RENDERING_VISIBLE WavefieldVisualPlugin :
    public gazebo::VisualPlugin,
    public Ogre::RenderTargetListener
  {
    /// \brief Destructor.
    public: virtual ~WavefieldVisualPlugin();

    /// \brief Constructor.
    public: WavefieldVisualPlugin();

    /// \brief Load the plugin.
    public: virtual void Load(
      gazebo::rendering::VisualPtr _visual,
      sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// internal
    /// \brief Called every PreRender event.
    private: void OnPreRender();

    /// internal
    /// \brief Called every Render event.
    private: void OnRender();

    /// internal
    /// \brief Setup Ogre objects for reflection/refraction
    private: void SetupReflectionRefraction();

    /// internal
    /// \brief Get new cameras
    private: std::vector<Ogre::Camera*> NewCameras();

    /// internal
    /// \brief Callback for gztopic "~/world_stats".
    ///
    /// \param[in] _msg World statistics message.
    private: void OnStatsMsg(ConstWorldStatisticsPtr &_msg);

    /// internal
    /// \brief Update the vertex shader parameters.
    private: void SetShaderParams();

    /// internal
    /// \brief Hide/Show objects for reflection/refraction render
    ///        eg. hide objects above water for refraction
    ///            hide objects below water for reflection
    ///            unhide all objects after texture is rendered
    private: virtual void preRenderTargetUpdate(
                 const Ogre::RenderTargetEvent& rte);
    private: virtual void postRenderTargetUpdate(
                 const Ogre::RenderTargetEvent& rte);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldVisualPluginPrivate> data;
  };
}

#endif
