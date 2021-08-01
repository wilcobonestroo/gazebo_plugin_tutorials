#ifndef _TRICYCLE_PLUGIN_HH_
#define _TRICYCLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
class TricyclePlugin : public ModelPlugin
{
  /// \brief Pointer to the model.
 private:
  physics::ModelPtr model;

  /// \brief Pointer to the joint.
 private:
  physics::JointPtr front_wheel_joint;
  physics::JointPtr steering_axel_joint;

    /// \brief A PID controller for the joint.
 private:
  common::PID pid_vel;
  common::PID pid_steer;

  common::Time time;

  bool left = true;

  /// \brief Listen to the update events
 private:
  event::ConnectionPtr updateConnection;

  /// \brief Constructor
 public:
  TricyclePlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    std::cerr << "Loading tricycle plugin...\n";

    // Safety check
    if (_model->GetJointCount() == 0) {
      std::cerr << "Did not see joints in the model. ";
      std::cerr << "Tricycle should have 4 joints.\n";
      return;
    }

    // Store the model pointer for convenience.
    this->model = _model;

    time = this->model->GetWorld()->SimTime();

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.

    // this->joint = _model->GetJoints()[0];
    this->front_wheel_joint = model->GetJoint("front_wheel_joint");
    this->steering_axel_joint = model->GetJoint("steering_axel_joint");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TricyclePlugin::OnUpdate, this));

    if (this->front_wheel_joint == 0) {
      std::cerr << "There should be a joint with the name [front_wheel_joint].\n";
      return;
    } else if (this->steering_axel_joint == 0) {
      std::cerr << "There should be a joint with the name [steering_axel_joint].\n";
      return;
    }

    // Setup a P-controller, with a gain of 0.1.
    this->pid_vel = common::PID(0.3, 0.1, 0.01);
    this->pid_steer = common::PID(1.2, 1.0, 1.0);

    // Apply the P-controller to the joint.
    this->model->GetJointController()->SetVelocityPID(
      this->front_wheel_joint->GetScopedName(), this->pid_vel);

    this->model->GetJointController()->SetPositionPID(
      this->steering_axel_joint->GetScopedName(), this->pid_steer);

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.

    this->model->GetJointController()->SetVelocityTarget(
      this->front_wheel_joint->GetScopedName(), 4.0);

    this->model->GetJointController()->SetPositionTarget(
      this->steering_axel_joint->GetScopedName(), 0.6);

  }

  public: void OnUpdate() {

    if ((this->model->GetWorld()->SimTime() - this->time).Double() > 5) {
      left = !left;
      this->time = this->model->GetWorld()->SimTime();

      if (left) {
        this->model->GetJointController()->SetPositionTarget(
          this->steering_axel_joint->GetScopedName(), 0.6);  // steering to the left
      } else {
        this->model->GetJointController()->SetPositionTarget(
          this->steering_axel_joint->GetScopedName(), -0.6);  // steering to the left
      }
    }
  }
};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TricyclePlugin)
}
#endif
