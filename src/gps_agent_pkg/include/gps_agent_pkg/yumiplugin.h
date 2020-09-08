/*
This is the PR2-specific version of the robot plugin.
*/
#pragma once

// Headers.
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
//#include <pr2_mechanism_model/chain.h>
#include "gps_agent_pkg/kdlchain.h"
#include <pr2_mechanism_model/robot.h>
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

// Superclass.
#include "gps_agent_pkg/robotplugin.h"
#include "gps_agent_pkg/controller.h"
#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/encodersensor.h"
#include "gps/proto/gps.pb.h"

namespace gps_control
{

class GPSYumiPlugin: public RobotPlugin, public controller_interface::Controller<hardware_interface::EffortJointInterface>
//class GPSYumiPlugin: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
private:
    // PR2-specific chain object necessary to construct the KDL chain.
    //pr2_mechanism_model::KdlChain passive_arm_chain_, active_arm_chain_;
    KdlChain passive_arm_chain_, active_arm_chain_;
    // This is a pointer to the robot state, which we get when initialized and have to keep after that.
    //pr2_mechanism_model::RobotState* robot_;
    // Passive arm joint state handlers.
    std::vector< hardware_interface::JointHandle > passive_arm_joint_states_;
    // Active arm joint state handlers.
    std::vector< hardware_interface::JointHandle > active_arm_joint_states_;
    // Passive arm joint states.
    //std::vector<pr2_mechanism_model::JointState*> passive_arm_joint_state_;
    // Active arm joint states.
    //std::vector<pr2_mechanism_model::JointState*> active_arm_joint_state_;
    // Passive arm joint names.
    std::vector<std::string> passive_arm_joint_names_;
    // Active arm joint names.
    std::vector<std::string> active_arm_joint_names_;
    // Active arm static friction %.
    std::vector<double> active_arm_stat_fric_percent;
    // Active arm static friction.
    std::vector<double> active_arm_stat_fric_;
    // Active arm dynamic friction.
    std::vector<double> active_arm_dyn_fric_;
    // Active arm friction torque.
    std::vector<double> active_arm_fric_trq_;
    // Time of last state update.
    ros::Time last_update_time_;
    // Counter for keeping track of controller steps.
    int controller_counter_;
    // Length of controller steps in ms.
    int controller_step_length_;
    // Controller frequency in Hz.
    int controller_period_ms_;
    // Controller period counter ms.
    int current_controller_period_ms_;
public:
    // Constructor (this should do nothing).
    GPSYumiPlugin();
    // Destructor.
    virtual ~GPSYumiPlugin();
    // Functions inherited from superclass.
    // This called by the superclass to allow us to initialize all the PR2-specific stuff.
    /* IMPORTANT: note that some sensors require a KDL chain to do FK, which we need the RobotState to get... */
    //virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& n);
    virtual bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
    // This is called by the controller manager before starting the controller.
    //virtual void starting();
    virtual void starting(const ros::Time& time);
    // This is called by the controller manager before stopping the controller.
    //virtual void stopping();
    virtual void stopping(const ros::Time& time);
    // This is the main update function called by the realtime thread when the controller is running.
    //virtual void update();
    virtual void update(const ros::Time& time, const ros::Duration& period);
    /* the pr2-specific update function should do the following:
       - perform whatever housekeeping is needed to note the current time.
       - update all sensors (this might be a no-op for vision, but for
         joint angle "sensors," they need to know the current robot state).
       - update the appropriate controller (position or trial) depending on
         what we're currently doing
       - if the controller wants to send something via a publisher, publish
         that at the end -- it will typically be a completion message that
         includes the recorded robot state for the controller's execution.
     */
    // Accessors.
    // Get current time.
    virtual ros::Time get_current_time() const;
    // Get current encoder readings (robot-dependent).
    virtual void get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const;
};

}
