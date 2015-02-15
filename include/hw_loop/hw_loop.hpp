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

// Joint Limits Includes
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "nav_msgs/Odometry.h"
    
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Empty.h"


// boost library ublas for matrix
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>    
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/operation.hpp>

#define SURGE 0
#define SWAY 1
#define HEAVE 2
#define YAW 3
#define PITCH 4
#define ROLL 5

namespace ublas = boost::numeric::ublas;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
// Sigint handler
void set_terminate_flag(int);

namespace UWEsub {
    class phoenix_hw_interface : public hardware_interface::RobotHW {
    public:
        phoenix_hw_interface();
        ~phoenix_hw_interface();

        void terminate(void);
        static bool terminate_flag;
        static bool safe;
        bool are_thrusters_scaled_down(void);
        bool panic( std_srvs::Empty::Request& request,  std_srvs::Empty::Response& response);


        /// DOF feedback subscriber callbacks
        void sub_callback(const nav_msgs::Odometry::ConstPtr& message);

    private:
        /// DOF feedback subscribers
        ros::Subscriber feedback_fused;

        /// virtual panic button
        ros::ServiceServer panic_stopper;
        /// Write to 'physical' thruster drivers
        int write(void);

        /// DOF update functions
        void update(const ros::TimerEvent&);
        void panic_loop(const ros::TimerEvent&);
        double loop_hz_;
        /// DOF update timers
        ros::Timer timer_update;

        /// Thruster linearisation, Allocation and scaling

        int scale_commands(void);
        int get_maximum_command(void);
        double max_command;
        bool thrusters_scaled_down;
        int thrust_to_command(void);
        int get_linearisation_parameter(void);
        std::vector<double> positive_linearisation_m;
        std::vector<double> negative_linearisation_m;
        std::vector<double> positive_linearisation_c;
        std::vector<double> negative_linearisation_c;

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
        ublas::vector<double> cmd;
        ublas::vector<double> pos;
        ublas::vector<double> vel;
        ublas::vector<double> eff;

        /// write command to hardware thruster drivers
        ublas::vector<double> write_command;
        ublas::vector<double> thrust;

        //ros::Duration control_period_;

        // double state_x_position, state_x_velocity;

        //Controller Limits:
        urdf::Model urdf;

        std::vector<JointLimits> limits;
        std::vector<SoftJointLimits> soft_limits;
        int get_controller_limits(void);
        joint_limits_interface::EffortJointSoftLimitsInterface jnt_limits_interface_;
    };
}

#endif
