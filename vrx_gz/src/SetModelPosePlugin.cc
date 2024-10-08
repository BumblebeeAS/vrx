#include <mutex>
#include <string>

#include <gz/math/Matrix4.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

#include "SetModelPosePlugin.hh"
#include <gz/transport/Node.hh>

using namespace gz;
using namespace vrx;

/// \brief Private SetModelPosePlugin data class.
class SetModelPosePlugin::Implementation {
  /// \brief Callback function called when receiving a new pose message.
  /// \param[in] _msg The new pose to set.
public:
  void OnSetPose(const gz::msgs::Pose &_msg);

  /// \brief Protect some member variables used in the callback.
public:
  std::mutex mutex;

  /// \brief Gz transport node
public:
  gz::transport::Node node;

  /// \brief Model entity.
public:
  gz::sim::Model model;

  /// \brief New pose to set.
public:
  gz::math::Pose3d newPose;
};

//////////////////////////////////////////////////
SetModelPosePlugin::SetModelPosePlugin()
    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>()) {}

//////////////////////////////////////////////////
void SetModelPosePlugin::Configure(
    const sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr) {
  // Get the model entity
  this->dataPtr->model = sim::Model(_entity);

  // Parse <topic> if available.
  std::string topic = "/model/set_pose";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  // Subscribe to the topic
  this->dataPtr->node.Subscribe(topic,
                                &SetModelPosePlugin::Implementation::OnSetPose,
                                this->dataPtr.get());

  gzdbg << "SetModelPosePlugin subscribed to topic: " << topic << std::endl;
}

//////////////////////////////////////////////////
void SetModelPosePlugin::PreUpdate(const sim::UpdateInfo &,
                                   sim::EntityComponentManager &_ecm) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->model.Entity() == sim::kNullEntity)
    gzdbg << "SetModelPosePlugin model not found." << std::endl;
  return;

  // Set the new pose of the model
  this->dataPtr->model.SetWorldPoseCmd(_ecm, this->dataPtr->newPose);
}

//////////////////////////////////////////////////
void SetModelPosePlugin::Implementation::OnSetPose(const gz::msgs::Pose &_msg) {
  std::lock_guard<std::mutex> lock(this->mutex);

  // Convert the message to a math::Pose3d
  this->newPose = gz::math::Pose3d(
      _msg.position().x(), _msg.position().y(), _msg.position().z(),
      _msg.orientation().w(), _msg.orientation().x(), _msg.orientation().y(),
      _msg.orientation().z());
}

GZ_ADD_PLUGIN(SetModelPosePlugin, sim::System,
              SetModelPosePlugin::ISystemConfigure,
              SetModelPosePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::SetModelPosePlugin, "vrx::SetModelPosePlugin")
