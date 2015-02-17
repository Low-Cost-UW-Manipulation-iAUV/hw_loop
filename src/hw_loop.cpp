/**********************************************************************
*   Copyright 2014 GNU License
*   This node is the hardware controller for the UWEsub Phoenix
*   or the BMT submarine.
*
*
*   further modified by Raphael Nagel
*   first.lastname (#) posteo de
*   14/Oct/2014
*
*   To do:
*       watchdog timer
*///////////////////////////////////////////////////////////////////////



#include "ros/ros.h"
#include <signal.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>
#include "hw_loop/hw_loop.hpp"

#define ODOM 0
#define POOL 1      // Or any other frame that is not special
#define BL_LIKE 2   // frame that is aligned with the current orientation and its origin is at the point when this frame started being published
namespace ublas = boost::numeric::ublas;

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace UWEsub {

phoenix_hw_interface::phoenix_hw_interface() {
    goal_frame = "pool";
    ///set the controller output to 0
    cmd.resize(6);
    cmd[0] = 0.0;
    cmd[1] = 0.0;
    cmd[2] = 0.0;
    cmd[3] = 0.0;
    cmd[4] = 0.0;
    cmd[5] = 0.0;

    // and the controller input / current position
    pos.resize(6);
    pos[0] = 0.0;
    pos[1] = 0.0;
    pos[2] = 0.0;
    pos[3] = 0.0;
    pos[4] = 0.0;
    pos[5] = 0.0;

    vel.resize(6);
    vel[0] = 0.0;
    vel[1] = 0.0;
    vel[2] = 0.0;
    vel[3] = 0.0;
    vel[4] = 0.0;
    vel[5] = 0.0;

    eff.resize(6);
    eff[0] = 0.0;
    eff[1] = 0.0;
    eff[2] = 0.0;
    eff[3] = 0.0;
    eff[4] = 0.0;
    eff[5] = 0.0;

    // resize the callback data vector and reset it to 0;
    callback_pos.resize(6,0.0);

    sequence = 0;
    terminate_flag = false;
    safe = false;
    // set the pool origin = [0,0,0,0,0,0]
    init_pool_origin();

    /// get the parameters
    get_allocation_matrix();
    get_maximum_command();
    get_linearisation_parameter();

    /// Create a panic command line panic button that can be used like this: rosservice call stop
    panic_stopper = nh_.advertiseService("/stop", &phoenix_hw_interface::panic, this);
    if(panic_stopper) {
        ROS_INFO("DOF_5_hw_loop: Panic button online");
    } else {
        ROS_ERROR("DOF_5_hw_loop: Panic Button not functioning");
    }        

    /// Advertise the I-SO transformer service:
    set_iso_reference_service = nh_.advertiseService("/hw_loop/setIsoReference", &phoenix_hw_interface::setIsoReference, this);
    // Initialise the URDF file that holds all the limits and load more params from parameterserver
    get_controller_limits();

    /// connect and register the joint state interface for the 6 DOF
    hardware_interface::JointStateHandle state_handle_x("x", &pos[0], &vel[0], &eff[0]);
    joint_state_interface.registerHandle(state_handle_x);

    hardware_interface::JointStateHandle state_handle_y("y", &pos[1], &vel[1], &eff[1]);
    joint_state_interface.registerHandle(state_handle_y);

    hardware_interface::JointStateHandle state_handle_z("z", &pos[2], &vel[2], &eff[2]);
    joint_state_interface.registerHandle(state_handle_z);

    hardware_interface::JointStateHandle state_handle_yaw("yaw", &pos[3], &vel[3], &eff[3]);
    joint_state_interface.registerHandle(state_handle_yaw);

    hardware_interface::JointStateHandle state_handle_pitch("pitch", &pos[4], &vel[4], &eff[4]);
    joint_state_interface.registerHandle(state_handle_pitch);

    hardware_interface::JointStateHandle state_handle_roll("roll", &pos[5], &vel[5], &eff[5]);
    joint_state_interface.registerHandle(state_handle_roll); 

    /// register the joint_state_interface
    registerInterface(&joint_state_interface);



    /// connect and register the joint effort interface for the 6 potentially controlled DOF
    hardware_interface::JointHandle pos_handle_x(joint_state_interface.getHandle("x"), &cmd[0]);
    jnt_eff_interface.registerHandle(pos_handle_x);

    hardware_interface::JointHandle pos_handle_y(joint_state_interface.getHandle("y"), &cmd[1]);
    jnt_eff_interface.registerHandle(pos_handle_y);

    hardware_interface::JointHandle pos_handle_z(joint_state_interface.getHandle("z"), &cmd[2]);
    jnt_eff_interface.registerHandle(pos_handle_z);

    hardware_interface::JointHandle pos_handle_yaw(joint_state_interface.getHandle("yaw"), &cmd[3]);
    jnt_eff_interface.registerHandle(pos_handle_yaw);

    hardware_interface::JointHandle pos_handle_pitch(joint_state_interface.getHandle("pitch"), &cmd[4]);
    jnt_eff_interface.registerHandle(pos_handle_pitch);

    hardware_interface::JointHandle pos_handle_roll(joint_state_interface.getHandle("roll"), &cmd[5]);
    jnt_eff_interface.registerHandle(pos_handle_roll);              

    /// register the joint effort interface
    registerInterface(&jnt_eff_interface);

    // initialise the controller limits and register them
    joint_limits_interface::EffortJointSoftLimitsHandle limit_handle_x(pos_handle_x, limits.at(0), soft_limits.at(0));
    jnt_limits_interface_.registerHandle(limit_handle_x);

    joint_limits_interface::EffortJointSoftLimitsHandle limit_handle_y(pos_handle_y, limits.at(1), soft_limits.at(1));

    joint_limits_interface::EffortJointSoftLimitsHandle limit_handle_z(pos_handle_z, limits.at(2), soft_limits.at(2));
    jnt_limits_interface_.registerHandle(limit_handle_z);

    joint_limits_interface::EffortJointSoftLimitsHandle limit_handle_yaw(pos_handle_yaw, limits.at(3), soft_limits.at(3));
    jnt_limits_interface_.registerHandle(limit_handle_yaw);

    joint_limits_interface::EffortJointSoftLimitsHandle limit_handle_pitch(pos_handle_pitch, limits.at(4), soft_limits.at(4));
    jnt_limits_interface_.registerHandle(limit_handle_pitch);
    
    joint_limits_interface::EffortJointSoftLimitsHandle limit_handle_roll(pos_handle_roll, limits.at(5), soft_limits.at(5));
    jnt_limits_interface_.registerHandle(limit_handle_roll);


    /// Subscribe to the Feedback Signal
    feedback_fused = nh_.subscribe<nav_msgs::Odometry>("odometry/filtered", 1, &phoenix_hw_interface::sub_callback, this);

    ///Initialise the controller manager
    ROS_INFO("DOF_5_hw_loop: Loading the controller manager");
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    /// Set up the real time safe publisher
    thruster_driver_command_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh_, "motorVal", 1) );

    /// Get the required variables from the parameter server and set standard values if not available
    loop_hz_ = 0;
    if (!nh_.getParam("/thruster_interface/update_rate", loop_hz_)) {

        ROS_ERROR("DOF_5_hw_loop: Could not find update rate, assuming 50. \n");
        loop_hz_ = 50;
        nh_.setParam("thruster_interface/update_rate", loop_hz_);
    }

    /// Set up the control loop by creating a timer and a connected callback
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    timer_update = nh_.createTimer(update_freq, &phoenix_hw_interface::update, this);
}

/** Destructor: 
*/
phoenix_hw_interface::~phoenix_hw_interface() {}

int phoenix_hw_interface::get_controller_limits(void) {

    // read the urdf from the parameter server
    if (!urdf.initParam("/robot_description")){
        ROS_ERROR("Failed to read the urdf from the parameter server");
        return -1;
    }
    // Get the list of joints we want to limit
    std::vector<std::string> temp_joints;
    if (!nh_.getParam("/Controller/joints_with_limits", temp_joints)) {

        ROS_ERROR("DOF_5_hw_loop: Could not find Controllers joints with limits \n");
        return -1;
    }

    // Load all limits into:
    limits.resize(temp_joints.size());
    soft_limits.resize(temp_joints.size());

    int x = 0;
    for (std::vector<std::string>::iterator it = temp_joints.begin(); it != temp_joints.end(); ++it, x++) {
        
        // get the joint name from the param server BUT look up the data from the URDF FILE
        boost::shared_ptr<const urdf::Joint> urdf_joint = urdf.getJoint(*it);
        const bool urdf_limits_ok = getJointLimits(urdf_joint, limits.at(x));
        const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits.at(x) );
        
        // Get the updates from the yaml file
        const bool rosparam_limits_ok = getJointLimits(*it, nh_, limits.at(x));
    }

}


/**terminate(): Will try to stop the thrusters before shutting down
                Should send at lreast 3 messages with 0 commands before we shutdown
*/
void phoenix_hw_interface::terminate(void) {
    timer_update.stop();
    for (int x = 0; x < write_command.size(); x++) {
        write_command[x] = 0;
    }
    write();
    write();

    while ( write() != EXIT_SUCCESS ) {
        usleep(100);
        for (int x = 0; x < write_command.size(); x++) {
            write_command[x] = 0;
        }
        ROS_ERROR("DOF_5_hw_loop: still trying to stop the thrusters, then shutting down");
    } 
    ROS_ERROR("DOF_5_hw_loop: shutting down, set the thrusters to 0"); 
    safe = true;
}

/** panic(): the service call that will stop all thrusters
        usage: rosservice call stop
*/
bool phoenix_hw_interface::panic( std_srvs::Empty::Request& request,  std_srvs::Empty::Response& response) {
    /// Stopping the ros::timer
    timer_update.stop();
    for (int x = 0; x < write_command.size(); x++) {
        write_command[x] = 0;
    }

    while ( write() != EXIT_SUCCESS ) {
        usleep(100);
        for (int x = 0; x < write_command.size(); x++) {
            write_command[x] = 0;
        }
        ROS_ERROR("DOF_5_hw_loop: still trying to stop the thrusters");
    }

    ROS_ERROR("DOF_5_hw_loop: You have panic stopped the controller. All thrusters have stopped and the hardware loop needs restarting");
    // Just to be safe change to the panic_loop which just sends 0s out.
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    timer_update = nh_.createTimer(update_freq, &phoenix_hw_interface::panic_loop, this);

}

/** setIsoReference(): Sets the current position (in pool) as the reference for the ISO transformation procedure, stores it in iso_reference_pose
*/
bool phoenix_hw_interface::setIsoReference(ros_control_iso::string_ok::Request& request, ros_control_iso::string_ok::Response& response) {
    geometry_msgs::PoseStamped temp_in;
    geometry_msgs::PoseStamped temp_out;
    // Get Convert the Pose with Covariance Stamped into PoseStamped
    temp_in.header = callback_message.header;
    temp_in.pose = callback_message.pose.pose;
    try {
        listener.waitForTransform(callback_message.header.frame_id, "/pool", ros::Time(0), ros::Duration(1));
        listener.transformPose("/pool", temp_in, temp_out);
    } catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
      return 0;
    } 
    temp_out.header.stamp = ros::Time(0);
    iso_reference_pose = temp_out;
    //Tell the little state machine in read() what to do
    goal_frame = request.data;
    ROS_INFO("iso_reference_pose [pool]: x: %f, y: %f, z: %f", iso_reference_pose.pose.position.x,iso_reference_pose.pose.position.y,iso_reference_pose.pose.position.z);
    ROS_INFO("goal:frame: %s", goal_frame.c_str());
    return 1;
}

/** panic_loop(): once a panic stop has been registered this loop will replace update()
        It will continously send a value of 0 to the thrusters.
*/
void phoenix_hw_interface::panic_loop(const ros::TimerEvent& event) {

    for (int x = 0; x < write_command.size(); x++) {
        write_command[x] = 0;
    }
    write();
}


/** sub_callback_...() listens to the corresponding DOF's feedback signal
The signal is stored unaltered.
*/
void phoenix_hw_interface::sub_callback(const nav_msgs::Odometry::ConstPtr& message) {
    callback_message = *message;
}

/** extract_6_DOF(): Extract the data from the Odometry message into a ublas::vector
*/
void phoenix_hw_interface::extract_6_DOF(ublas::vector<double>& extracted_data) {
    extracted_data[SURGE] = callback_message.pose.pose.position.x;
    extracted_data[SWAY] = callback_message.pose.pose.position.y;
    extracted_data[HEAVE] = callback_message.pose.pose.position.z;  
    
    // get the Quaternion into 
    tf::Quaternion q(callback_message.pose.pose.orientation.x, callback_message.pose.pose.orientation.y, callback_message.pose.pose.orientation.z, callback_message.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(extracted_data[ROLL], extracted_data[PITCH], extracted_data[YAW]);
    ROS_INFO("extracted: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",extracted_data[SURGE], extracted_data[SWAY], extracted_data[HEAVE], extracted_data[ROLL], extracted_data[PITCH], extracted_data[YAW]);
}

/** extract_6_DOF(): Extract the data from the Odometry message into a geometry_msgs::PoseStamped
*/
void phoenix_hw_interface::extract_6_DOF(const geometry_msgs::PoseStamped& stored_message, ublas::vector<double>& extracted_data) {
    extracted_data[SURGE] = stored_message.pose.position.x;
    extracted_data[SWAY] = stored_message.pose.position.y;
    extracted_data[HEAVE] = stored_message.pose.position.z;  
    
    // get the Quaternion into 
    tf::Quaternion q(stored_message.pose.orientation.x, stored_message.pose.orientation.y, stored_message.pose.orientation.z, stored_message.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(extracted_data[ROLL], extracted_data[PITCH], extracted_data[YAW]);
    ROS_INFO("extracted: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",extracted_data[SURGE], extracted_data[SWAY], extracted_data[HEAVE], extracted_data[ROLL], extracted_data[PITCH], extracted_data[YAW]);    
}


/** transform_for_controller_feedback(): Uses a pose reference to feed the controller
                                        Transforms the subs position in odom into the feedback frame
                                        extracts the position
*/
void phoenix_hw_interface::transform_for_controller_feedback(void) {
    // Convert the PoseWithCovarianceStamped into a simple PoseStamped
    geometry_msgs::PoseStamped pose_in;
    pose_in.header = callback_message.header;
    pose_in.pose = callback_message.pose.pose;

    geometry_msgs::PoseStamped pose_out;
    // find the subs pose in the pool frame
    try {
        listener.waitForTransform(pose_in.header.frame_id, "/feedback", ros::Time(0), ros::Duration(0.07));
        listener.transformPose("/feedback", pose_in, pose_out);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("transform for controller feedback: %s\n", ex.what()); //Print exception which was caught
      return;
    }
    ROS_INFO("iso_reference_pose: %f, %f, %f", iso_reference_pose.pose.position.x, iso_reference_pose.pose.position.y, iso_reference_pose.pose.position.z);

    // Get the data out of the odometry message
    extract_6_DOF(pose_out, pos);
}

void phoenix_hw_interface::init_pool_origin(void) {
    // Set a pose at the pool origin
    Pool_Origin.header.stamp = ros::Time(0);
    Pool_Origin.header.frame_id = "pool";
    Pool_Origin.pose.orientation.x = 0.0;
    Pool_Origin.pose.orientation.y = 0.0;
    Pool_Origin.pose.orientation.z = 0.0;
    Pool_Origin.pose.orientation.w = 1.0;


    Pool_Origin.pose.position.x = 0.0;
    Pool_Origin.pose.position.y = 0.0;
    Pool_Origin.pose.position.z = 0.0;
}

void phoenix_hw_interface::publish_feedback_frame(bool pool_corner) {
    // Get the current transform from pool to base_link - current pose of sub in pool
    tf::StampedTransform transform_found;
    try {
        listener.lookupTransform("/pool", "/base_link" , ros::Time(0), transform_found);        
    } catch (tf::TransformException &ex) {
      ROS_ERROR("Publish Feedback Frame%s",ex.what());
      ros::Duration(1.0).sleep();        
    }

    // Publish a new transform from pool->feedback
    tf::Transform transform;

    // Use the pool corner as reference point (0,0,0)- for normal operation
    if(pool_corner == true) {
        transform.setOrigin( tf::Vector3(Pool_Origin.pose.position.x,Pool_Origin.pose.position.y,Pool_Origin.pose.position.z) );
    
    // Use the iso reference position set when calling the service
    } else {
        transform.setOrigin( tf::Vector3(iso_reference_pose.pose.position.x,iso_reference_pose.pose.position.y,iso_reference_pose.pose.position.z) );
    }
    // use the rotation from pool->base_link for the rotation
    transform.setRotation(transform_found.getRotation() );

    //broadcast it as the transform from pool to feedback
    bc_feedback.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "/pool", "/feedback" ) );

}

void phoenix_hw_interface::read(void) {
    // if the goal frame is pool, 
    if (goal_frame == "pool") {
        publish_feedback_frame(true);
        transform_for_controller_feedback();   // For ISO we need a frame 
    } else if (goal_frame == "ISO") {
        publish_feedback_frame(false);
        transform_for_controller_feedback();
    } else {    //any other frame
        ROS_ERROR("goal_frame is unknown");
    }

}

/** Update():  the real action happens here: run the controllers and update the actuator output
*/
void phoenix_hw_interface::update(const ros::TimerEvent& event) {
    if(terminate_flag == true){
        terminate();
    } else {
        /// Update the time since the last update
        ros::Duration elapsed_time_ = ros::Duration(event.current_real - event.last_real);

        // Make sure the feedback is in the correct frame...
        read();

        ROS_INFO("hw_loop - pos: %f, %f, %f, %f, %f, %f", pos[0],pos[1],pos[2],pos[3],pos[4], pos[5]);

        /// Let the controller do its work
        controller_manager_->update(ros::Time::now(), elapsed_time_);

        // Joint Limits go here
        //jnt_limits_interface_.enforceLimits(elapsed_time_);

        ROS_INFO("hw_loop - cmd: %f, %f, %f, %f, %f, %f", cmd[0],cmd[1],cmd[2],cmd[3],cmd[4], cmd[5]);
        // find invidivdual thruster force demands from body frame force demands
        thruster_allocation();

        ROS_INFO("hw_loop - write_cmd: %f, %f, %f, %f, %f", write_command[0],write_command[1],write_command[2],write_command[3],write_command[4]);
        ROS_INFO("-----------------------");
        // calculate the thruster command from the force
        thrust_to_command();

        // scale forces down if thrusters are saturated
        scale_commands();

        // Write the new command to the motor drivers
        write();
    }
}


/** Write(): writes the command to the actual hardware driver
*       This is done by sending a message of type std_msgs::Float32MultiArray
*       to the ros topic motorVal.
*       When running on the BMT BRD then make sure there aren't more than 6 commands
*/
int phoenix_hw_interface::write(void) {
    // send the message in a realtime safe fashion
   if ( thruster_driver_command_publisher_ && thruster_driver_command_publisher_->trylock() ) {
        // resize the array
        thruster_driver_command_publisher_->msg_.data.clear();
        thruster_driver_command_publisher_->msg_.data.resize(write_command.size(),0);
        // fill the message with the commands to be written
        for (int x = 0; x < write_command.size(); x++) {
            thruster_driver_command_publisher_->msg_.data[x] = write_command[x];
        }
        // send it off
        thruster_driver_command_publisher_->unlockAndPublish();
        return EXIT_SUCCESS;
   } else {
        ROS_ERROR("5_DOF_hw_loop: could not send message to the thruster driver");
        return EXIT_FAILURE;
   }
}

/** get_allocation_matrix(): gets the parameters from the server
        It gets the allocation matrix and stores it
        Additionally it will resize the write_command vector 
        which holds the commands to be directly send to the thrusters.
        This is done here as the matrix encodes the number of DOFs (# of columns)
        to be controlled as well as the number of thrusters to do so (# of rows).

*/
int phoenix_hw_interface::get_allocation_matrix(void) {
    double matrix_size_rows;
    double matrix_size_columns;
    std::vector<double> temp_matrix;

    /// get the size of the allocation_matrix. The number of rows corresponds to the number of DOFs
    if (!nh_.getParam("/thruster_interface/allocation_matrix/rows", matrix_size_rows)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find allocation_matrix row size\n");
        return EXIT_FAILURE;
    }

    /// get the size of the allocation_matrix. The number of columns corresponds to the number of thrusters.
    if (!nh_.getParam("/thruster_interface/allocation_matrix/columns", matrix_size_columns)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find allocation_matrix columns size\n");
        return EXIT_FAILURE;
    }

    /// prepare the allocation matrix to hold them
    allocation_matrix.clear();
    allocation_matrix.resize(matrix_size_rows, matrix_size_columns, false);
    
    // clear and resize the vector of patched together rows
    temp_matrix.clear();
    temp_matrix.resize((matrix_size_rows * matrix_size_columns),0);
    
    // load the allocation matrix data in vector form
    if (!nh_.getParam("/thruster_interface/allocation_matrix/data", temp_matrix)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find allocation_matrix on the parameter server\n");
        return EXIT_FAILURE;

    // store the allocation matrix in matrix form
    } else {
        ROS_INFO("5_DOF_hw_loop: Allocation Matrix loaded:");
        // write to the matrix row by row
        for (int x = 0; x < matrix_size_rows; x++) {  // rows
            for (int y = 0; y < matrix_size_columns; y++) {  // columns
                allocation_matrix (x,y) = temp_matrix[x * matrix_size_columns + y];
            } 
        } 
        std::cout << allocation_matrix << "\n";
    }

    // set the number of thrusters being controlled:
    // for the commands written to thrusters
    write_command.clear();
    write_command.resize(matrix_size_rows,0);

    // for the thrust [N] demanded of the thrusters
    thrust.clear();
    thrust.resize(matrix_size_rows,0); 

    return EXIT_SUCCESS;     
}

/** get_linearisation_parameter(): gets the parameters from the server
*/
int phoenix_hw_interface::get_linearisation_parameter(void) {
    if (!nh_.getParam("/thruster_interface/linearisation/positive/m", positive_linearisation_m)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find positive linearisation 'm' on the parameter server\n");
        return EXIT_FAILURE;
    }
    if (!nh_.getParam("/thruster_interface/linearisation/positive/c", positive_linearisation_c)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find positive linearisation 'c' on the parameter server\n");
        return EXIT_FAILURE;
    }

    if (!nh_.getParam("/thruster_interface/linearisation/negative/m", negative_linearisation_m)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find negative linearisation 'm' on the parameter server\n");
        return EXIT_FAILURE;
    }
    if (!nh_.getParam("/thruster_interface/linearisation/negative/c", negative_linearisation_c)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find negative linearisation 'c' on the parameter server\n");
        return EXIT_FAILURE;
    }        
}

/** get_maximum_command(): gets the parameters from the server
*/
int phoenix_hw_interface::get_maximum_command(void) {   
    if (!nh_.getParam("/thruster_interface/max_command", max_command)) {
        ROS_ERROR("5_DOF_hw_loop: Could not find maximum thruster command\n");
        return EXIT_FAILURE;
    }
}

/** scale_thrust() scales down all thrust commands ( [-100:100] usually) when thrusters are saturated.
        Based on DOI:978-1-4799-0997-1/13$31.00 by Miskovic et al.
*/
int phoenix_hw_interface::scale_commands(void) {
    double largest_command = 0;
    // find the largest command in the range
    largest_command = *std::max_element(write_command.begin(), write_command.end());
    // if that command saturates the thrusters scale down all commands.
    if (largest_command > max_command) {
        thrusters_scaled_down = true;
        double scale_down;
        if (largest_command == 0) {
            scale_down = 1;
        } else {
            scale_down = max_command/largest_command;  // calculate the scaling factor
        }
        
        // scale all commands
        for(int x = 0; x < write_command.size(); x++) {
            write_command[x] = write_command[x] * scale_down;
        }
        ROS_INFO("hw_loop: scaling down commands");
    } else {
        thrusters_scaled_down = false;
    }
}

/** are_thrusters_scaled_down(): are the thrusters currently scaled down?
*/
bool phoenix_hw_interface::are_thrusters_scaled_down(void) {
    return thrusters_scaled_down;
}

/** thrust_to_command(): translates the force (N) demand from the controller into the command signal [-100:100]
        linearises the thruster force using Fossen's simplified model: force=b*|command|*command
        --> for positive : command = sqrt(force/b_f)
        --> negative: command =  -1* sqrt(force/b_b)
*/
int phoenix_hw_interface::thrust_to_command(void) {
    // clear the thrust vector. THe allocation_matrix x axis size corresponds to the number of thrusters.
    
    write_command.clear();
    write_command.resize(allocation_matrix.size1(),0);

    for (int x = 0; x < write_command.size(); x++) {
        // handle the positive force
        if (thrust[x] > 0) {
            write_command[x] = thrust[x] * positive_linearisation_m[x] + positive_linearisation_c[x]; //sqrt(thrust[x]/positive_linearisation[x]);

            // handle negative force
        } else if (thrust[x] < 0) {
            write_command[x] = thrust[x] * negative_linearisation_m[x] + negative_linearisation_c[x]; //sqrt(abs(thrust[x])/negative_linearisation[x]) * -1;
        } else {
            write_command[x] = 0.0;
        }
    }

}

/** thruster_allocation(): turns the body frame force demand into commands for individual thrusters
*/
int phoenix_hw_interface::thruster_allocation(void) {
    ublas::vector<double> temp_cmd;
    temp_cmd.clear();
    temp_cmd.resize(6, false);
    
    // clear the thrust vector. THe allocation_matrix x axis size corresponds to the number of thrusters.
    thrust.clear();
    thrust.resize(allocation_matrix.size1(),0);

    // build a temporary vector to allow for boost::ublas matrix calculations below:
    for (int x = 0; x < 6; x++) {
        temp_cmd[x] = cmd[x];
    }

    // calculate the product of allocation_matrix * temp_command = write command per thruster. This is in boost::ublas
    ublas::axpy_prod(allocation_matrix, temp_cmd, thrust, true);
    return EXIT_SUCCESS;       
}

}  // end of namespace


bool UWEsub::phoenix_hw_interface::terminate_flag = 0;
bool UWEsub::phoenix_hw_interface::safe = 0;

/** set_terminate_flag(): sets the static terminate_flag inside the phoenix_hw_interface.
                            The set flag causes the object to initiate a safe shutdown which sends
                            0 commands to the thrusters.
*/
void set_terminate_flag(int sig) {
    UWEsub::phoenix_hw_interface::terminate_flag = true;
    while(UWEsub::phoenix_hw_interface::safe == false){
        usleep(100);
    }
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "DOF_5_hw_loop");
    ros::NodeHandle nh;
    /// An Async spinner creates another thread which will handle the event of this node being executed.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // create the instance of the class
    UWEsub::phoenix_hw_interface hw_loop;

    // register the 
    signal(SIGINT, set_terminate_flag);
    ros::spin();


    ROS_INFO("5_DOF_hw_loop: Shutting down hardware interface");
}
