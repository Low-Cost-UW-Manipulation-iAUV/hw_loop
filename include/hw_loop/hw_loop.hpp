/* Author: Raphael Nagel
    Desc: ros_control_interface for testing purpose of the ros_control_iso controller
    Date: 02/Sept/2014
*/

#ifndef _DOF_5_hw_loop_
#define _DOF_5_hw_loop_

#include <stdio.h>
#include <vector>
#include <sstream>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "geometry_msgs/Vector3.h"
    
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Empty.h"


// boost library ublas for matrix
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>    
#include <boost/numeric/ublas/io.hpp>

namespace ublas = boost::numeric::ublas;

namespace UWEsub {
    class phoenix_hw_interface : public hardware_interface::RobotHW {
    public:
        phoenix_hw_interface();
        ~phoenix_hw_interface();

        bool are_thrusters_scaled_down(void);
        bool panic( std_srvs::Empty::Request& request,  std_srvs::Empty::Response& response);


        /// DOF feedback subscriber callbacks
        void sub_callback_x(const geometry_msgs::Vector3::ConstPtr&);
        void sub_callback_y(const geometry_msgs::Vector3::ConstPtr&);
        void sub_callback_z(const geometry_msgs::Vector3::ConstPtr&);
        void sub_callback_yaw(const geometry_msgs::Vector3::ConstPtr&);
        void sub_callback_pitch(const geometry_msgs::Vector3::ConstPtr&);
        void sub_callback_roll(const geometry_msgs::Vector3::ConstPtr&);

    private:
        /// DOF feedback subscribers
        ros::Subscriber sub_x;
        ros::Subscriber sub_y;
        ros::Subscriber sub_z;
        ros::Subscriber sub_yaw;
        ros::Subscriber sub_pitch;
        ros::Subscriber sub_roll;

        /// virtual panic button
        ros::ServiceServer panic_stopper;
        /// Write to 'physical' thruster drivers
        int write(void);

        /// DOF update functions
        void update(const ros::TimerEvent&);

        /// DOF update timers
        ros::Timer timer_update;

        /// Thruster linearisation, Allocation and scaling

        int scale_commands(void);
            int get_maximum_command(void);
            double max_command;
            bool thrusters_scaled_down;
        int thrust_to_command(void);
            int get_linearisation_parameter(void);
            double positive_linearisation;
            double negative_linearisation;

        int thruster_allocation(void);
            int get_allocation_matrix(void);
            ublas::matrix<double> allocation_matrix;

        ros::NodeHandle nh_;
        boost::scoped_ptr <realtime_tools::RealtimePublisher <std_msgs::Float32MultiArray> > thruster_driver_command_publisher_;
        unsigned int sequence;

        /// ros_controller interface
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::EffortJointInterface jnt_eff_interface;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        /// controller_manager
        double cmd[6];
        double pos[6];
        double vel[6];
        double eff[6];

        /// write command to hardware thruster drivers
        ublas::vector<double> write_command;
        ublas::vector<double> thrust;

        //ros::Duration control_period_;

        // double state_x_position, state_x_velocity;

    };
}

#endif
