#include "gps_agent_pkg/yumiplugin.h"
#include "gps_agent_pkg/positioncontroller.h"
#include "gps_agent_pkg/trialcontroller.h"
#include "gps_agent_pkg/encodersensor.h"
#include "gps_agent_pkg/util.h"

namespace gps_control {

// // Plugin constructor.
GPSYumiPlugin::GPSYumiPlugin()
{
    NUM_JOINTS = 7;
    // Some basic variable initialization.
    controller_counter_ = 0;
    controller_step_length_ = 50; // do not use this when using controller_frequency_
    // controller_period_ms_ = 50;
    controller_period_ms_ = 10;
    current_controller_period_ms_ = 0;

    active_arm_stat_fric_.resize(NUM_JOINTS);
    active_arm_stat_fric_.clear();
    active_arm_stat_fric_percent.resize(NUM_JOINTS);
    active_arm_stat_fric_percent.clear();
    // active_arm_stat_fric_.push_back(2.43);
    // active_arm_stat_fric_.push_back(2.76);
    // active_arm_stat_fric_.push_back(1.11);
    // active_arm_stat_fric_.push_back(0.52);
    // active_arm_stat_fric_.push_back(0.52);
    // active_arm_stat_fric_.push_back(0.52);
    // // active_arm_stat_fric_.push_back(1.0);
    // active_arm_stat_fric_.push_back(0.52);

    active_arm_stat_fric_percent.push_back(0.8);
    active_arm_stat_fric_percent.push_back(0.8);
    active_arm_stat_fric_percent.push_back(0.6);
    active_arm_stat_fric_percent.push_back(0.6);
    active_arm_stat_fric_percent.push_back(0.5);
    active_arm_stat_fric_percent.push_back(0.5);
    active_arm_stat_fric_percent.push_back(0.5);

    active_arm_stat_fric_.push_back(2.43);
    active_arm_stat_fric_.push_back(2.76);
    active_arm_stat_fric_.push_back(1.11);
    active_arm_stat_fric_.push_back(0.52);
    active_arm_stat_fric_.push_back(0.4);
    active_arm_stat_fric_.push_back(0.2);
    // active_arm_stat_fric_.push_back(1.0);
    active_arm_stat_fric_.push_back(0.4);

    active_arm_dyn_fric_.resize(NUM_JOINTS);
    active_arm_dyn_fric_.clear();
    active_arm_dyn_fric_.push_back(1.06);
    active_arm_dyn_fric_.push_back(1.09);
    active_arm_dyn_fric_.push_back(0.61);
    active_arm_dyn_fric_.push_back(0.08);
    active_arm_dyn_fric_.push_back(0.08);
    active_arm_dyn_fric_.push_back(0.08);
    active_arm_dyn_fric_.push_back(0.08);

    active_arm_fric_trq_.resize(NUM_JOINTS);
    active_arm_fric_trq_.clear();

}
//
// // Destructor.
GPSYumiPlugin::~GPSYumiPlugin()
{
//     // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize the object and store the robot state.
bool GPSYumiPlugin::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
// bool GPSYumiPlugin::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& n)
{
    // Variables.
    // std::string root_name, active_tip_name, passive_tip_name, robot_desc_string;

    ROS_INFO_STREAM("GPSYumiPlugin init begin");

    // // Create FK solvers.
    // // Get the name of the root.
    // if(!n.getParam("root_name", root_name)) {
    //     ROS_ERROR("Property root_name not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    //
    // // Get active and passive arm end-effector names.
    // if(!n.getParam("active_tip_name", active_tip_name)) {
    //     ROS_ERROR("Property active_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    // if(!n.getParam("passive_tip_name", passive_tip_name)) {
    //     ROS_ERROR("Property passive_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    //
    // // Create the robot model from param server.
    // // TODO:the robot model may be available from the hardware interface given by the
    // // controller manager. Change this later
    // if(!n.getParam("robot_description", robot_desc_string)) {
    //     ROS_ERROR("Property robot_description not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    //
    //
    // // Create active arm chain.
    // if(!active_arm_chain_.init(hw, robot_desc_string, root_name, active_tip_name)) {
    //     ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), active_tip_name.c_str());
    //     return false;
    // }

    // // Create passive arm chain.
    // if(!passive_arm_chain_.init(hw, robot_desc_string, root_name, passive_tip_name)) {
    //     ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), passive_tip_name.c_str());
    //     return false;
    // }

    // only for testing, delete this
    //std::string param_name_temp="/yumi/active_arm_joint_name_1";
    //std::string param_value_temp;
    //n.getParam(param_name_temp, param_value_temp);
    //ROS_INFO_STREAM(param_name_temp+": "+param_value_temp);

    // // Create KDL chains, solvers, etc.
    // // KDL chains.
    // passive_arm_chain_.toKDL(passive_arm_fk_chain_);
    // active_arm_chain_.toKDL(active_arm_fk_chain_);
    //
    // // Pose solvers.
    // passive_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(passive_arm_fk_chain_));
    // active_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(active_arm_fk_chain_));
    //
    // // Jacobian sovlers.
    // passive_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(passive_arm_fk_chain_));
    // active_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(active_arm_fk_chain_));
    //
    // // some fwd kin - temp calculations - delete after use
    // KDL::Frame temp_pose_;
    // KDL::JntArray temp_jnt_array_;
    // //Eigen::Vector3d pos;
    // //Eigen::Matrix3d rot;
    // double pos, rot;
    // temp_jnt_array_.resize(7);
    // // temp_jnt_array_(0)=-1.147903;
    // // temp_jnt_array_(1)=-1.462062;
    // // temp_jnt_array_(2)=0.831475;
    // // temp_jnt_array_(3)=0.573341;
    // // temp_jnt_array_(4)=2.039243;
    // // temp_jnt_array_(5)=1.563640;
    // // temp_jnt_array_(6)=-2.156354;
    //
    // temp_jnt_array_(0)=-0.79005;
    // temp_jnt_array_(1)=-1.79093;
    // temp_jnt_array_(2)=0.77403;
    // temp_jnt_array_(3)=0.74101;
    // temp_jnt_array_(4)=2.11053;
    // temp_jnt_array_(5)=1.07796;
    // temp_jnt_array_(6)=-2.01606;
    //
    // active_arm_fk_solver_->JntToCart(temp_jnt_array_, temp_pose_);
    // for (unsigned i = 0; i < 3; i++){
    //     pos = temp_pose_.p(i);
    //     ROS_INFO_STREAM("fk pos x0:"+to_string(i)+":"+to_string(pos));
    //   }
    // for (unsigned j = 0; j < 3; j++)
    //     for (unsigned i = 0; i < 3; i++){
    //         rot = temp_pose_.M(i,j);
    //         ROS_INFO_STREAM("fk rot x0:"+to_string(i)+to_string(j)+":"+to_string(rot));
    //       }
    //
    // // inserted pos
    // // temp_jnt_array_(0)=-1.256637;
    // // temp_jnt_array_(1)=-1.462586;
    // // temp_jnt_array_(2)=1.106015;
    // // temp_jnt_array_(3)=0.301418;
    // // temp_jnt_array_(4)=2.185327;
    // // temp_jnt_array_(5)=1.244071;
    // // temp_jnt_array_(6)=-2.070659;
    //
    // temp_jnt_array_(0)=-1.32847;
    // temp_jnt_array_(1)=-1.57387;
    // temp_jnt_array_(2)=1.01143;
    // temp_jnt_array_(3)=0.26778;
    // temp_jnt_array_(4)=2.26350;
    // temp_jnt_array_(5)=1.38524;
    // temp_jnt_array_(6)=-2.01606;
    //
    //
    // active_arm_fk_solver_->JntToCart(temp_jnt_array_, temp_pose_);
    // for (unsigned i = 0; i < 3; i++){
    //     pos = temp_pose_.p(i);
    //     ROS_INFO_STREAM("fk pos tgt:"+to_string(i)+":"+to_string(pos));
    //   }
    // for (unsigned j = 0; j < 3; j++)
    //     for (unsigned i = 0; i < 3; i++){
    //         rot = temp_pose_.M(i,j);
    //         ROS_INFO_STREAM("fk rot tgt:"+to_string(i)+to_string(j)+":"+to_string(rot));
    //       }

    // Pull out joint states.
    int joint_index;
    active_arm_joint_states_.clear();
    // Put together joint states for the active arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this active joint exists.
        std::string joint_name;
        std::string param_name = std::string("/yumi/active_arm_joint_name_" + to_string(joint_index));
        //if(!n.getParam(param_name.c_str(), joint_name)){
        if(!n.getParam(param_name, joint_name)){
            ROS_INFO_STREAM("param name: "+param_name);
            ROS_INFO_STREAM("joint name: "+joint_name);
            break;
        }
        // Push back the joint state and name.
        hardware_interface::JointHandle jointHandle = hw->getHandle(joint_name);
        active_arm_joint_states_.push_back(jointHandle);
        //TODO: any other check possible?
        //if (active_arm_joint_states_[joint_index] == NULL) // TODO this may not be correct
        //    ROS_INFO_STREAM("jointState: " + joint_name + " is null");

        active_arm_joint_names_.push_back(joint_name);
        ROS_INFO_STREAM("active arm joint names"+joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.

    // if (active_arm_fk_chain_.getNrOfJoints() != active_arm_joint_states_.size())
    if (NUM_JOINTS != active_arm_joint_states_.size())
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(NUM_JOINTS));
        ROS_INFO_STREAM("num_joint_state: " + to_string(active_arm_joint_states_.size()));
        ROS_ERROR("Number of joints in the active arm FK chain does not match the number of joints in the active arm joint state!");
        return false;
    }

    // Put together joint states for the passive arm.
    joint_index = 1;
    passive_arm_joint_states_.clear();
    while (true)
    {
        // Check if the parameter for this passive joint exists.
        std::string joint_name;
        std::string param_name = std::string("/yumi/passive_arm_joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name, joint_name))
            break;

        // Push back the joint state and name.
        hardware_interface::JointHandle jointHandle = hw->getHandle(joint_name);
        passive_arm_joint_states_.push_back(jointHandle);
        //if (passive_arm_joint_states_[joint_index] == NULL) // TODO this may not be correct
        //    ROS_INFO_STREAM("jointState: " + joint_name + " is null");
        passive_arm_joint_names_.push_back(joint_name);
        //ROS_DEBUG_STREAM("passive arm joint names: "<<joint_index<<passive_arm_joint_names_[joint_index]);
        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.
    // if (passive_arm_fk_chain_.getNrOfJoints() != passive_arm_joint_states_.size())
    if (NUM_JOINTS != passive_arm_joint_states_.size())
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(NUM_JOINTS));
        ROS_INFO_STREAM("num_joint_state: " + to_string(passive_arm_joint_states_.size()));
        ROS_ERROR("Number of joints in the passive arm FK chain does not match the number of joints in the passive arm joint state!");
        return false;
    }

    // Allocate torques array.
    active_arm_torques_.resize(NUM_JOINTS);
    passive_arm_torques_.resize(NUM_JOINTS);
    //
    // Initialize ROS subscribers/publishers, sensors, and position controllers.
    // Note that this must be done after the FK solvers are created, because the sensors
    // will ask to use these FK solvers!
    initialize(n);
    ROS_INFO_STREAM("GPSYumiPlugin init end");
    // Tell the PR2 controller manager that we initialized everything successfully.
    return true;
}

// This is called by the controller manager before starting the controller.
void GPSYumiPlugin::starting(const ros::Time& time)
// void GPSYumiPlugin::starting()
{
    ROS_INFO_STREAM("GPSYumiPlugin starting begin");
    // Get current time.
    //last_update_time_ = robot_->getTime();
    last_update_time_ = time;
    ROS_INFO_STREAM("Plugin starting time: "+to_string(last_update_time_));
    controller_counter_ = 0;
    current_controller_period_ms_ = 0;

    // Reset all the sensors. This is important for sensors that try to keep
    // track of the previous state somehow.
    //for (int sensor = 0; sensor < TotalSensorTypes; sensor++)
    for (int sensor = 0; sensor < 1; sensor++)
    {
        sensors_[sensor]->reset(this,last_update_time_);
    }

    // Reset position controllers.
    passive_arm_controller_->reset(last_update_time_);
    active_arm_controller_->reset(last_update_time_);

    // Reset trial controller, if any.
    if (trial_controller_ != NULL) trial_controller_->reset(last_update_time_);
    ROS_INFO_STREAM("GPSYumiPlugin starting end");
}

// This is called by the controller manager before stopping the controller.
void GPSYumiPlugin::stopping(const ros::Time& time)
// void GPSYumiPlugin::stopping()
{
    // Nothing to do here.
}

// This is the main update function called by the realtime thread when the controller is running.
void GPSYumiPlugin::update(const ros::Time& time, const ros::Duration& period)
// void GPSYumiPlugin::update()
{
    ROS_INFO_STREAM("GPSYumiPlugin update begin");
    // Get current time.
    //last_update_time_ = robot_->getTime();
    ros::Duration dur = time - last_update_time_;
    last_update_time_ = time;
    current_controller_period_ms_ += (int)(dur.toNSec()*1e-6);
    //ROS_INFO_STREAM("Plugin update time: "+to_string(last_update_time_));

    // Check if this is a controller step based on the current controller frequency.
    controller_counter_++;
    //if (controller_counter_ >= controller_step_length_) controller_counter_ = 0;
    //bool is_controller_step = (controller_counter_ == 0);
    if (current_controller_period_ms_ >= controller_period_ms_){
      controller_counter_ = 0;
      //ROS_INFO_STREAM("controller update tick"<<current_controller_period_ms_);
      current_controller_period_ms_ = 0;
    }
    bool is_controller_step = (current_controller_period_ms_ == 0);

    // Update the sensors and fill in the current step sample.
    update_sensors(last_update_time_,is_controller_step);

    // Update the controllers.
    update_controllers(last_update_time_,is_controller_step);

    // Store the torques.
    // compute friction torque here

    for (unsigned i = 0; i < active_arm_joint_states_.size(); i++) {
        // double speed;
        // double sign = 0;
        // speed = active_arm_joint_states_[i].getVelocity();
        // if (speed > 0.0) sign = 1.0;
        // else if (speed < 0.0) sign = -1.0;
        // else sign = 0.0;
        // active_arm_fric_trq_[i] = sign*active_arm_stat_fric_[i]*0.5 + speed*active_arm_dyn_fric_[i]*0;
        // if (active_arm_torques_[i]==0) active_arm_fric_trq_[i] = 0;
        active_arm_fric_trq_[i] = (controller_counter_%2)?(active_arm_stat_fric_[i]*active_arm_stat_fric_percent[i]):(active_arm_stat_fric_[i]*-active_arm_stat_fric_percent[i]);
        // active_arm_joint_states_[i].setCommand(active_arm_torques_[i]);
        active_arm_joint_states_[i].setCommand(active_arm_torques_[i] + active_arm_fric_trq_[i]);
        // active_arm_joint_states_[i].setCommand(0.0);
      }
    // ROS_INFO_STREAM_THROTTLE(0.5,"active arm joint torques: "<<active_arm_torques_);


    for (unsigned i = 0; i < passive_arm_joint_states_.size(); i++)
        //passive_arm_joint_states_[i].setCommand(passive_arm_torques_[i]);
        passive_arm_joint_states_[i].setCommand(0);
    //ROS_DEBUG_STREAM_THROTTLE(2,"passive arm joint torques: "<<passive_arm_torques_);
    //ROS_INFO_STREAM("GPSYumiPlugin update end");
}

// Get current time.
ros::Time GPSYumiPlugin::get_current_time() const
{
    // return last_update_time_;
}

// Get current encoder readings (robot-dependent).
void GPSYumiPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const
{
    if (arm == gps::AUXILIARY_ARM)
    {
        if (angles.rows() != passive_arm_joint_states_.size())
            angles.resize(passive_arm_joint_states_.size());
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = passive_arm_joint_states_[i].getPosition();
        //ROS_INFO_STREAM_THROTTLE(1,"aux arm joint angles: "<<angles);
    }
    else if (arm == gps::TRIAL_ARM)
    {
        if (angles.rows() != active_arm_joint_states_.size())
            angles.resize(active_arm_joint_states_.size());
        for (unsigned i = 0; i < angles.size(); i++){
            angles(i) = active_arm_joint_states_[i].getPosition();
            //ROS_INFO_STREAM("trial arm joint angle "+to_string(i)+": "+to_string(angle(i)));}
            //ROS_DEBUG_STREAM_THROTTLE(10,"trial arm joint angle %d: %f",i,angles(i));
          }
          // ROS_INFO_STREAM_THROTTLE(1,"trial arm joint angles: "<<angles);
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!",arm);
    }
}

}

// Register controller to pluginlib

// PLUGINLIB_DECLARE_CLASS(gps_agent_pkg, GPSYumiPlugin,
// 						gps_control::GPSYumiPlugin,
// 						controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(gps_control::GPSYumiPlugin, controller_interface::ControllerBase)
