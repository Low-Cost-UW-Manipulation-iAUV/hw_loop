/**********************************************************************
*   Copyright 2014 GNU License
*	This node is the hardware controller for the UWEsub Phoenix
*	or the BMT submarine.
*
*
*   further modified by Raphael Nagel
*   first.lastname (#) posteo de
*   14/Oct/2014
*
*	To do:
*		watchdog timer
*///////////////////////////////////////////////////////////////////////


#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <algorithm>

#include "ros/ros.h"
#include <sstream>
#include <stdio.h>

#include <DOF_5_hw_loop/DOF_5_hw_loop.hpp>

namespace UWEsub {
	phoenix_hw_interface::phoenix_hw_interface() {
		///set the controller output to 0
		cmd[0] = 0.0;
		cmd[1] = 0.0;
		cmd[2] = 0.0;
		cmd[3] = 0.0;
		cmd[4] = 0.0;
		cmd[5] = 0.0;

		// and the controller input / current position
		pos[0] = 0.0;
		pos[1] = 0.0;
		pos[2] = 0.0;
		pos[3] = 0.0;
		pos[4] = 0.0;
		pos[5] = 0.0;

		sequence = 0;

		/// get the parameters
		if(get_allocation_matrix() == EXIT_FAILURE) {

			return EXIT_FAILURE;
		}

		if(get_maximum_command() == EXIT_FAILURE) {

			return EXIT_FAILURE;
		}

		if(get_linearisation_parameter() == EXIT_FAILURE) {

			return EXIT_FAILURE;
		}


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



		/// connect and register the joint position interface for the 6 potentially controlled DOF
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


		/// Subscribe to the Feedback Signal
		sub_x = nh_.subscribe<geometry_msgs::Vector3>("feedback/x",1, &phoenix_hw_interface::sub_callback_x, this);
		sub_y = nh_.subscribe<geometry_msgs::Vector3>("feedback/y",1, &phoenix_hw_interface::sub_callback_y, this);
		sub_z = nh_.subscribe<geometry_msgs::Vector3>("feedback/z",1, &phoenix_hw_interface::sub_callback_z, this);
		sub_yaw = nh_.subscribe<geometry_msgs::Vector3>("feedback/yaw",1, &phoenix_hw_interface::sub_callback_yaw, this);
		sub_pitch = nh_.subscribe<geometry_msgs::Vector3>("feedback/pitch",1, &phoenix_hw_interface::sub_callback_pitch, this);
		sub_xroll = nh_.subscribe<geometry_msgs::Vector3>("feedback/roll",1, &phoenix_hw_interface::sub_callback_roll, this);


		/// Create a panic command line panic button that can be used like this: rosservice call stop
		panic_stopper = nh_.advertiseService("stop", &identification_server::panic, this);
		if(panic_stopper) {
			ROS_INFO("DOF_5_hw_loop: Panic button online");
		} else {
			ROS_ERROR("DOF_5_hw_loop: Panic Button not functioning");
		}

		///Initialise the controller manager
		ROS_INFO("DOF_5_hw_loop: Loading the controller manager");
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));


CHANGE ME!!!
		/// Set up the real time safe publisher
	    thruster_driver_command_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(n, "motorVal", 1) );

		/// Get the required variables from the parameter server and set standard values if not available
		double loop_hz_ = 0;
		if (!nh_.getParam("/thruster_interface/update_rate", loop_hz_)) {

			ROS_ERROR("DOF_5_hw_loop: Could not find update rate, assuming 50. \n");
			loop_hz_ = 50;
			nh_.setParam("thruster_interface/update_rate", loop_hz_);
		}

		/// Set up the control loop by creating a timer and a connected callback
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		timer_update = nh_.createTimer(update_freq, &phoenix_hw_interface::update, this);
	}

	/** Destructor
	*/
	phoenix_hw_interface::~phoenix_hw_interface() {}

	/** panic(): the service call that will stop all thrusters
			usage: rosservice call stop
	*/
	void phoenix_hw_interface::panic(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

		/// Stopping the ros::timer
		non_realtime_loop_.stop();

		cmd[0] = 0;
		cmd[1] = 0;
		cmd[2] = 0;
		cmd[3] = 0;
		cmd[4] = 0;
		cmd[5] = 0;
		write();
		ROS_ERROR("DOF_5_hw_loop: You have panic stopped the controller. All thrusters have stopped and the hardware loop needs restarting");
	}

	/** sub_callback_...() listens to the corresponding DOF's feedback signal
	The signal is stored unaltered.
	*/
	void phoenix_hw_interface::sub_callback_x(const geometry_msgs::Vector3::ConstPtr& message) {
		pos[0] = message->x;
	}

	void phoenix_hw_interface::sub_callback_y(const geometry_msgs::Vector3::ConstPtr& message) {
		pos[1] = message->x;
	}

	void phoenix_hw_interface::sub_callback_z(const geometry_msgs::Vector3::ConstPtr& message) {
		pos[2] = message->x;
	}

	void phoenix_hw_interface::sub_callback_yaw(const geometry_msgs::Vector3::ConstPtr& message) {
		pos[3] = message->x;
	}

	void phoenix_hw_interface::sub_callback_pitch(const geometry_msgs::Vector3::ConstPtr& message) {
		pos[4] = message->x;
	}

	void phoenix_hw_interface::sub_callback_roll(const geometry_msgs::Vector3::ConstPtr& message) {
		pos[5] = message->x;
	}


	/** Update():  the real action happens here: run the controllers and update the actuator output
	*/
	void phoenix_hw_interface::update(const ros::TimerEvent& event) {
		/// Update the time since the last update
		ros::Duration elapsed_time_ = ros::Duration(event.current_real - event.last_real);

		/// Let the controller do its work
		controller_manager_->update(ros::Time::now(), elapsed_time_);

		// find invidivdual thruster force demands from body frame force demands
		thruster_allocation();

		// calculate the thruster command from the force
		thrust_to_command();

		// scale forces down if thrusters are saturated
		scale_commands();

		// Write the new command to the motor drivers
		write();
	}


	/** Write(): writes the command to the actual hardware driver
	*		This is done by sending a message of type std_msgs::Float32MultiArray
	*		to the ros topic motorVal.
	*		When running on the BMT BRD then make sure there aren't more than 6 commands
	*/
	int phoenix_hw_interface::write(void) {
		// send the message in a realtime safe fashion
	   if(thruster_driver_command_publisher_ && thruster_driver_command_publisher_->trylock()) {
			
			// fill the message with the commands to be written
			for(int x = 0; x < write_command.size(); x++) {
				thruster_driver_command_publisher_->msg_.data[x] = write_command[x];
			}
			//send it off
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

		/// get the dynamics parameters
		if (!nh_.getParam("/thruster_interface/allocation_matrix/rows", matrix_size_rows)) {

			ROS_ERROR("5_DOF_hw_loop: Could not find allocation_matrix row size\n");
			return EXIT_FAILURE;
		}

		/// get the dynamics parameters
		if (!nh_.getParam("/thruster_interface/allocation_matrix/columns", matrix_size_columns)) {

			ROS_ERROR("5_DOF_hw_loop: Could not find allocation_matrix columns size\n");
			return EXIT_FAILURE;
		}

		/// prepare the allocation matrix to hold them
		allocation_matrix.clear();
		allocation_matrix.resize(matrix_size_rows, matrix_size_columns, false);

		if (!nh_.getParam("/thruster_interface/allocation_matrix/allocation_matrix", allocation_matrix)) {

			ROS_ERROR("5_DOF_hw_loop: Could not find allocation_matrix on the parameter server\n");
			return EXIT_FAILURE;
		}

		// set the number of thrusters being controlled:

		write_command.clear();
		write_command.resize(matrix_size_rows);		


	}

	/** get_linearisation_parameter(): gets the parameters from the server
	*/
	int phoenix_hw_interface::get_linearisation_parameter(void) {
		
		if (!nh_.getParam("/thruster_interface/linearisation/positive", positive_linearisation)) {

			ROS_ERROR("5_DOF_hw_loop: Could not find positive linearisation on the parameter server\n");
			return EXIT_FAILURE;
		}

		if (!nh_.getParam("/thruster_interface/linearisation/negative", negative_linearisation)) {

			ROS_ERROR("5_DOF_hw_loop: Could not find negative linearisation on the parameter server\n");
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
		if(largest_command > max_command) {
			thrusters_scaled_down = true;
			double scale_down;
			scale_down = max_command/largest_command; // calculate the scaling factor
			
			// scale all comman
			for(int x = 0; x < write_command.size(); x++) { 
				write_command[x] = write_command[x] * scale_down;
			}
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
		for(int x = 0; x< 5; x++) {

			// handle the positive force
			if(cmd[x] > 0) {
				write_command[x] = sqrt(cmd[x]/positive_linearisation);

				// handle negative force
			} else if (cmd[x] < 0) {
				write_command[x] = sqrt(cmd[x]/negative_linearisation) * -1;
			}
		}
	}

	/** thruster_allocation(): turns the body frame force demand into commands for individual thrusters
	*/
	int phoenix_hw_interface::thruster_allocation(void) {
		std::vector<double> temp_cmd;
		temp_cmd.clear();
		temp_cmd.resize(6,0);

		// build a temporary vector to allow for boost::ublas matrix calculations below:
		for(int x = 0; x < 6; x++) {
			temp_cmd[x] = cmd[x];
		}
		// calculate the product of allocation_matrix * temp_command = write command per thruster. This is in boost::ublas
		axpy_prod(allocation_matrix, temp_cmd, write_command, true);
	}
} // namespace



int main(int argc, char **argv) {
	ros::init(argc, argv, "DOF_5_hw_loop");
	ros::NodeHandle nh;
	/// An Async spinner creates another thread which will handle the event of this node being executed.
	ros::AsyncSpinner spinner(7);
	spinner.start();
	DOF_5_hw_loop::phoenix_hw_interface hw_loop;
	ros::spin();

	ROS_INFO("5_DOF_hw_loop: Shutting down hardware interface");

}
