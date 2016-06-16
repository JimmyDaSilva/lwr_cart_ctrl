// lwr_cart_ctrl - ISIR Tue Jun 14 10:39:51 2016
// Copyright (c) Jimmy Da Silva, All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library.

#ifndef __LWR_CART_CTRL_HPP__
#define __LWR_CART_CTRL_HPP__

// Orocos
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
// Eigen
#include <Eigen/Dense>
// RTT-ROS Utilities
#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <rtt_tf/tf_interface.h>


class LwrCartCtrl : public RTT::TaskContext{
    private:
        // RTT tf
        rtt_tf::TFInterface tf_;
    public:
        LwrCartCtrl(const std::string& name);
        virtual ~LwrCartCtrl(){};
        void updateHook();
        bool configureHook();
    protected:
        // Generic Model that uses ROS param
        rtt_ros_kdl_tools::ChainUtils arm;
        // Input ports
        RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                         port_joint_velocity_in,
                                         port_joint_torque_in;
        RTT::InputPort<geometry_msgs::Pose>  port_pose_goal_in;
                                         
        // Some input variables
        Eigen::VectorXd jnt_pos_in,
                        jnt_vel_in,
                        jnt_trq_in;
        // Output ports
        RTT::OutputPort<Eigen::VectorXd> port_joint_position_cmd_out,
                                         port_joint_velocity_cmd_out,
                                         port_joint_torque_cmd_out;
        // Some output variables
        Eigen::VectorXd jnt_pos_cmd_out,
                        jnt_vel_cmd_out,
                        jnt_trq_cmd_out;
                        
        // Transform between tip_link and command frame
        KDL::Frame f_ati_to_tip;
        
        // Gains
        Eigen::VectorXd Kd, Kc;
        

};

#endif
