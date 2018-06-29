/*!
 * mixed_initiative_teleop_node.cpp
 * Copyright (c) 2014, Manolis Chiou
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*!

@mainpage
  Joystick teleoperation node for use within the mixed initiative framework. The user can operate the robot in
  teleoperation mode and change on the fly autonomy level/mode. Also a stop button is implimented.
  It was ment to be used with an Xbox 360 joystick but should work with any joystick.
<hr>

@section usage Usage
@par
@verbatim
$ mixed_initiative_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b /teleop/cmd_vel: will publish to /teleop/cmd_vel a geometry_msgs/Twist.msg type message to drrobot_player.
 For differential robots, linear.x is forward/backward speed (m/sec), and angular.z (rad/sec)is the angular speed.
<hr>
*/


#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

class JoystickTeleop
{

public:
    JoystickTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void flipper_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);

    ros::NodeHandle nh_;

    int linear_axis_, angular_axis_, control_button_, stop_button_, auto_button_, teleop_button_, enable_vel_button_, lighton_button_, lightoff_button_, bluelighton_button_, bluelightoff_button_;
    int axis_flipper_, button_flipper_front_down_, button_flipper_front_up_, button_flipper_rear_down_, button_flipper_rear_up_;
    double linear_scaling_, angular_scaling_, scale_flipper_, scale_flipper_rear_, scale_flipper_front_;

    std::vector<std::string> flipper_joint_names_;
    trajectory_msgs::JointTrajectoryPoint current_flipper_joints_;


    ros::Publisher vel_pub_, mode_pub_, light_pub_, bluelight_pub_ , jointstate_pub_, flipper_joints_pub_;

    ros::Subscriber joy_sub_, flipper_joint_states_sub_;

};


JoystickTeleop::JoystickTeleop()
{
    // Default movement axis
    nh_.param("axis_linear", linear_axis_, 1);
    nh_.param("axis_angular", angular_axis_, 0);

    // Default scaling parameters
    nh_.param("scale_angular", angular_scaling_, 0.8);
    nh_.param("scale_linear", linear_scaling_, 0.2);

    //Default buttons for Xbox 360 joystick.
    nh_.param("teleop_button", teleop_button_, 3); // Y button
    nh_.param("auto_button", auto_button_, 0);     // A button
    nh_.param("stop_button", stop_button_, 4);     // LB button
    nh_.param("enable_vel_button", enable_vel_button_, 5);     // RB button
    nh_.param("bluelightoff_button", bluelightoff_button_, 1);     // B button
    nh_.param("bluelighton_button", bluelighton_button_, 2);     // X button
    nh_.param("lightoff_button", lightoff_button_, 7);     // start button
    nh_.param("lighton_button", lighton_button_, 6);     // back/select button

    // default buttons and axis for flippers
    nh_.param("axis_flipper", axis_flipper_, 4);
    nh_.param("scale_flipper", scale_flipper_, 0.6);
    nh_.param("button_flipper_front_down", button_flipper_front_down_, 0); //6,7;
    nh_.param("button_flipper_front_up", button_flipper_front_up_, 1);
    nh_.param("button_flipper_rear_down", button_flipper_rear_down_, 2);
    nh_.param("button_flipper_rear_up", button_flipper_rear_up_, 3);
    nh_.param("scale_flipper_front", scale_flipper_front_, 0.6);
    nh_.param("scale_flipper_rear", scale_flipper_rear_, 0.6);


    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 5);
    mode_pub_ = nh_.advertise<std_msgs::Int8>("/control_mode", 5);
    light_pub_ = nh_.advertise<std_msgs::Bool>("/light", 5);
    bluelight_pub_ = nh_.advertise<std_msgs::Bool>("/bluelight", 5);
    jointstate_pub_ = nh_.advertise<sensor_msgs::JointState>("jointstate_cmd", 1);
    flipper_joints_pub_= nh_.advertise<trajectory_msgs::JointTrajectory>("//flipper/flipper_traj_controller/command", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 2, &JoystickTeleop::joyCallback, this);
    flipper_joint_states_sub_ = nh_.subscribe<control_msgs::JointTrajectoryControllerState>("/flipper/flipper_traj_controller/state", 1, &JoystickTeleop::flipper_joint_statesCallback, this);

}

void JoystickTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist cmd_vel;
    std_msgs::Int8 mode;
    std_msgs::Bool bluelight, light;
    sensor_msgs::JointState jsm;
    trajectory_msgs::JointTrajectory flipper_traj;
    trajectory_msgs::JointTrajectoryPoint flipper_desired_joint_states;


    if (joy->buttons.size() > enable_vel_button_ && joy->buttons[enable_vel_button_])
    {


        // movement commands
        cmd_vel.linear.x = linear_scaling_ * joy->axes[linear_axis_];
        cmd_vel.angular.z = angular_scaling_ * joy->axes[angular_axis_];

        vel_pub_.publish(cmd_vel);
    }

    else
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;

        vel_pub_.publish(cmd_vel);
    }

    // autonomy mode choice
    if (joy->buttons[stop_button_]){
        mode.data=0;
        mode_pub_.publish(mode);
    }
    if (joy->buttons[teleop_button_]){
        mode.data=1;
        mode_pub_.publish(mode);
    }
    if (joy->buttons[auto_button_]){
        mode.data=2;
        mode_pub_.publish(mode);
    }

    // blue lights control
    if (joy->buttons[bluelighton_button_])
    {
        bluelight.data = true;
        bluelight_pub_.publish(bluelight);
    }

    if (joy->buttons[bluelightoff_button_])
    {
        bluelight.data = false;
        bluelight_pub_.publish(bluelight);
    }
    // lights control
    if (joy->buttons[lighton_button_])
    {
        light.data = true;
        light_pub_.publish(light);
    }

    if (joy->buttons[lightoff_button_])
    {
        light.data = false;
        light_pub_.publish(light);
    }


    // flipper control
    jsm.header.frame_id = "flippers_front";
    jsm.header.stamp = ros::Time::now();
    jsm.name.push_back("flippers_front");
    jsm.position.push_back(joy->axes[axis_flipper_] * scale_flipper_);
    jointstate_pub_.publish(jsm);


    // not sure at all what this bit from taurob original code does

    ros::Duration dur(0.5);

    for(int i=0; i < flipper_joint_names_.size(); i++)
    {
        flipper_traj.joint_names.push_back(flipper_joint_names_[i]);
        if (i == 0)
        {
            flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) +
                        joy->buttons[button_flipper_front_up_] * -scale_flipper_front_ +
                        joy->buttons[button_flipper_front_down_] * scale_flipper_front_);
        }
        else
        {
            flipper_desired_joint_states.positions.push_back(
                        current_flipper_joints_.positions.at(i) +
                        joy->buttons[button_flipper_rear_up_] * -scale_flipper_rear_ +
                        joy->buttons[button_flipper_rear_down_] * scale_flipper_rear_);
        }
    }

    flipper_desired_joint_states.time_from_start=dur;
            flipper_traj.header.stamp = ros::Time::now();
            flipper_traj.points.push_back(flipper_desired_joint_states);
            flipper_joints_pub_.publish(flipper_traj);

}

void JoystickTeleop::flipper_joint_statesCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    flipper_joint_names_= msg->joint_names;
    current_flipper_joints_= msg->actual;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "taurob_mi_teleop_node");
    JoystickTeleop joystick_teleop;

    ros::Rate r(20); // 20 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
