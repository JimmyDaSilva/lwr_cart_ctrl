#include "lwr_cart_ctrl/lwr_cart_ctrl.hpp"
#include <rtt_roscomm/rtt_rostopic.h>

LwrCartCtrl::LwrCartCtrl(const std::string& name):
RTT::TaskContext(name), tf_(this), Kd(6), Kc(6)
{
    // Here you can add your ports, properties and operations
    // Meanwhile, GenericArmController initialize the Arm() object, i.e. your model
    // and sets
    // and GenericController add the basic orocos ports
    this->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");
    this->addPort("JointTorque",port_joint_torque_in).doc("Current joint torques");

    this->addPort("JointPositionCommand",port_joint_position_cmd_out).doc("Command joint positions");
    this->addPort("JointVelocityCommand",port_joint_velocity_cmd_out).doc("Command joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Command joint torques");
    
    this->addPort("PoseGoalIn",port_pose_goal_in).doc("Command joint torques");
    
    this->addProperty("Kd", Kd);
    this->addProperty("Kc", Kc);
    
    Kd[0] = 0.7;
    Kd[1] = 0.7;
    Kd[2] = 0.7;
    Kd[3] = 0.7;
    Kd[4] = 0.7;
    Kd[5] = 0.7;   
    
    Kc[0] = 1000.0;
    Kc[1] = 1000.0;
    Kc[2] = 1000.0;
    Kc[3] = 200.0;
    Kc[4] = 50.0;
    Kc[5] = 50.0;

}

bool LwrCartCtrl::configureHook()
{
    if(!this->arm.init())
    {
        RTT::log(RTT::Fatal)
        << "Could not initialize arm, make sure roscore is launched"
        " as well as tip_link, root_link and robot_description"
        << RTT::endlog();
    }

    jnt_pos_in.setZero(arm.getNrOfJoints());
    jnt_vel_in.setZero(arm.getNrOfJoints());
    jnt_trq_in.setZero(arm.getNrOfJoints());

    jnt_pos_cmd_out.setZero(arm.getNrOfJoints());
    jnt_vel_cmd_out.setZero(arm.getNrOfJoints());
    jnt_trq_cmd_out.setZero(arm.getNrOfJoints());

    port_joint_position_cmd_out.setDataSample(jnt_pos_cmd_out);
    port_joint_velocity_cmd_out.setDataSample(jnt_vel_cmd_out);
    port_joint_torque_cmd_out.setDataSample(jnt_trq_cmd_out);
    
    port_pose_goal_in.createStream(rtt_roscomm::topic("/topic_name"));
    
    bool ready = tf_.ready();
    while(!tf_.canTransform("/ati_link","/epingle_tip_link")){
      usleep(1000000*1);
      printf("Waiting.......tf no ready\n");
    }
    geometry_msgs::TransformStamped tform = tf_.lookupTransform("/ati_link","/epingle_tip_link");
    tf::transformMsgToKDL(tform.transform, f_ati_to_tip);

    return true;
}

void LwrCartCtrl::updateHook()
{
    // Read status from robot
    port_joint_position_in.read(jnt_pos_in);
    port_joint_velocity_in.read(jnt_vel_in);

    // Update Internal model
    this->arm.setState(jnt_pos_in,jnt_vel_in);
    this->arm.updateModel();
    
    // Read the desired pose
    geometry_msgs::Pose pose_out;
    if (port_pose_goal_in.read(pose_out) == RTT::NoData){
      return;
    }
    
    KDL::Frame frame_out;
    tf::poseMsgToKDL(pose_out, frame_out);
//     printf("pose_out is (%f, %f, %f) (%f, %f, %f, %f)\n", pose_out.position.x, pose_out.position.y, pose_out.position.z, pose_out.orientation.x, pose_out.orientation.y, pose_out.orientation.z, pose_out.orientation.w);
    
    // Read the current pose
    KDL::Frame f_base_to_ati;
    std::string frame_name("ati_link");
    geometry_msgs::Pose pose_in;
    f_base_to_ati = this->arm.getSegmentPosition(frame_name);
    tf::poseKDLToMsg(f_base_to_ati, pose_in);
//     printf("base_to_ati is (%f, %f, %f) (%f, %f, %f, %f)\n", pose_in.position.x, pose_in.position.y, pose_in.position.z, pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
    tf::poseKDLToMsg(f_ati_to_tip, pose_in);
//     printf("ati_to_tip is (%f, %f, %f) (%f, %f, %f, %f)\n", pose_in.position.x, pose_in.position.y, pose_in.position.z, pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
    KDL::Frame frame_in = f_base_to_ati*f_ati_to_tip;
    tf::poseKDLToMsg(frame_in, pose_in);
//     printf("pose_in is (%f, %f, %f) (%f, %f, %f, %f)\n", pose_in.position.x, pose_in.position.y, pose_in.position.z, pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);
    
    // Compute twist between the two frames
    KDL::Twist kdl_twist = KDL::diff(frame_in, frame_out);
    Eigen::Matrix<double, 6, 1> xdiff, twist;
    tf::twistKDLToEigen(kdl_twist, xdiff);
    twist = xdiff;
    
    // clamp twist
    for(int i=0; i<6;i++){
      if (isnan(twist[i])){
        return;
      }
      if (std::abs(twist[i]) > 0.01 ){
        twist[i] = 0.01 * ((twist[i] > 0) - (twist[i] < 0));
      }
    }
    
    for(int i=0; i<6; i++){
      for(int j=0; j<7; j++){
        if (isnan(this->arm.getJacobian().data(i,j))){
          return;
        }
      }
    }
    
    // Compute the command 
    Eigen::VectorXd qcmd;
    qcmd = this->arm.getJacobian().data.transpose()*Kc.asDiagonal()*twist;
    
    
//     KDL::Jacobian J = this->arm.getJacobian();
//     J.changeBase(frame_in.M.Inverse());
    
//     this->arm.getSegmentVelocity(frame_name);
    
//     qcmd = J.data.transpose()*(Kc.asDiagonal()*twist +Kd.asDiagonal()*xdiff );
    
    // Send command
    port_joint_torque_cmd_out.write(qcmd);
    
}

// Let orocos know how to create the component
ORO_CREATE_COMPONENT(LwrCartCtrl)
