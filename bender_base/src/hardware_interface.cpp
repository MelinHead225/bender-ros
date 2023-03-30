#include <boost/assign.hpp>
#include "bender_base/hardware_interface.h"


namespace bender_base
{

/*
  Constructor which initializes join_state, postition_joint, and velocity_joint interfaces
*/
BenderHardware::BenderHardware()
{   
    //Initialize all joint names
    ros::V_string joint_names = boost::assign::list_of
        ("wheel_rf_joint")("wheel_rh_joint")("wheel_lf_joint")("wheel_lh_joint")
        ("leg_lf_joint")("leg_rf_joint")("leg_lh_joint")("leg_rh_joint");

    /*
      For every joint name create a "joint state handle" with pointers to the 
      to the joint's position, velocity, and effort
    */
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
            &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);
        if (i >= 4) {
            hardware_interface::JointHandle joint_handle(
                joint_state_handle, &joints_[i].command);
            position_joint_interface_.registerHandle(joint_handle);
        } else {
            hardware_interface::JointHandle joint_handle(
                joint_state_handle, &joints_[i].command);
            velocity_joint_interface_.registerHandle(joint_handle);
        }
    }

    //register all interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);

    //set up a callback function to be called each time a message is received
    feedback_sub_ = nh_.subscribe("/bender_teensy_serial/feedback", 1, &BenderHardware::feedbackCallback, this);
    //crete a publisher on a ROS topic to which the program can publish messages containing commands for the joints
    cmd_drive_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/bender_teensy_serial/cmd_drive", 1);
	//append a new dimention
    cmd_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	cmd_msg_.layout.dim[0].label = "joint_targets";
	cmd_msg_.layout.dim[0].size = 4;
    cmd_msg_.layout.dim[0].stride = 1;

    // CAN setup
    std::string can_device = nh_.param<std::string>("can_device", "can0");
    can_node_names.push_back("wheel_rf_joint");
    can_node_names.push_back("wheel_rh_joint");
    can_node_names.push_back("wheel_lf_joint");
    can_node_names.push_back("wheel_lh_joint");
    const unsigned short id0(nh_.param("wheel_rf_joint_node_id", 0));
    const unsigned short id1(nh_.param("wheel_rh_joint_node_id", 1));
    const unsigned short id2(nh_.param("wheel_lf_joint_node_id", 2));
    const unsigned short id3(nh_.param("wheel_lh_joint_node_id", 3));
    //if unable to add the four joints of the robot ROS abort
    if ( !( canbus_.add_axis(id0, "wheel_rf_joint") &&
            canbus_.add_axis(id1, "wheel_rh_joint") &&
            canbus_.add_axis(id2, "wheel_lf_joint") &&
            canbus_.add_axis(id3, "wheel_lh_joint") ) ) {
        ROS_FATAL("Failed to create one or more axis. Aborting.\n");
    }
    //initialize driver
    can::ThreadedSocketCANInterfaceSharedPtr driver = 
        std::make_shared<can::ThreadedSocketCANInterface>();
    //handle initialization of driver failure
    if (!driver->init(can_device, 0, can::NoSettings::create()))
    {
        ROS_FATAL("Failed to initialize can_device at %s\n", can_device.c_str());
    }
    //create a state listener where if a state change occurs print error to console
    can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
        [&driver](const can::State& s) {
            std::string err;
            driver->translateError(s.internal_error, err);
            fprintf(stderr, "CAN Device error: %s, asio: %s.\n",
                err.c_str(), s.error_code.message().c_str());
        }
    );
    // Pass the SocketCAN handle to master
    canbus_.init(driver);
    std::this_thread::sleep_for(500ms);
    for (auto& name : can_node_names)
    {
        canbus_.clear_errors(canbus_.axis(name)); //clear any errors associated with the CAN node's axis
        canbus_.set_input_vel(canbus_.axis(name), 0.0f); //set velocity input 
        canbus_.set_axis_requested_state(canbus_.axis(name), AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);//set the state so that the axis will be contolled using a closed-loop control scheme
    }
}

//deconstructor which stops the motors and setting the axis state to idle
BenderHardware::~BenderHardware()
{
    for (auto& name : can_node_names)
    {
        canbus_.set_input_vel(canbus_.axis(name), 0.0f);
        canbus_.set_axis_requested_state(canbus_.axis(name), AxisState::AXIS_STATE_IDLE);
    }
}


void BenderHardware::read()
{
    // CAN Bus
    for (auto& name : can_node_names)
    {
        const odrive_can_ros::ODriveAxis this_axis = canbus_.axis(name);
        //if axis isn't active send ROS warning
        if (!this_axis.is_active_)
        {
            ROS_WARN_THROTTLE(10, "%s is inactive", name.c_str());
        }
        const int node_id = this_axis.node_id;
        joints_[node_id].position = this_axis.pos_enc_estimate * 2.0 * M_PI;
        joints_[node_id].velocity = this_axis.vel_enc_estimate * 2.0 * M_PI;
        joints_[node_id].effort = this_axis.idq_second;
    }

    // Serial
    boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock); //create a lock on the mutex
    if (feedback_msg_ && feedback_msg_lock) //make sure only one thread at a time can access the feedback_msg
    {
		if (!steering_joints_homed_) //if joints aren't yet homed
        {
            for (int i = 0; i < 4; i++) //set joint positions
            {
                steering_joints_home_[i] = feedback_msg_->position[i]; 
            }
            steering_joints_homed_ = true;
        }
        for (int i = 4; i < 8; i++) //update position, velocity, and effort values based on feedback
		{
            joints_[i].position = feedback_msg_->position[i-4] - steering_joints_home_[i-4];
			joints_[i].velocity = feedback_msg_->velocity[i-4];
			joints_[i].effort   = feedback_msg_->effort[i-4];  
		}
	}
    
}


void BenderHardware::write()
{
	// CAN Bus
    for (auto& name : can_node_names)
    {
        const odrive_can_ros::ODriveAxis this_axis = canbus_.axis(name);
        if (!this_axis.is_active_ && this_axis.last_msg_time_ms_ < 1000)
        {
            canbus_.set_input_vel(this_axis, 0.0f);
            canbus_.set_axis_requested_state(this_axis, AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        const int node_id = this_axis.node_id;
        float wheel_cmd = 0.0;
        // if (abs(joints_[node_id+4].position) - abs(joints_[node_id+4].command) <= 15*M_PI/180.0)
        {
            wheel_cmd = joints_[node_id].command / 2.0 / M_PI;
        }
        canbus_.set_input_vel(this_axis, wheel_cmd);
    }
    cmd_msg_.data.clear();
	for (int i = 4; i < 8; i++)
	{
        if (steering_joints_homed_) {
            cmd_msg_.data.push_back( (float) joints_[i].command + steering_joints_home_[i-4]);
        } else {
            cmd_msg_.data.push_back( (float) joints_[i].command);
        }
	}
	cmd_drive_pub_.publish(cmd_msg_);
}


void BenderHardware::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	// Update the feedback message pointer to point to the current message. Block
	// until the control thread is not using the lock.
	boost::mutex::scoped_lock lock(feedback_msg_mutex_);
	feedback_msg_ = msg;
}

}  // namespace bender_base
