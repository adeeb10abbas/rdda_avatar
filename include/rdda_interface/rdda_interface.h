#ifndef RDDA_NODE_H
#define RDDA_NODE_H

/* C++ headers */
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <algorithm>
#include <memory>

/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rdda_interfaces_types/msg/joint_commands.hpp"
#include "rdda_interfaces_types/msg/joint_states.hpp"
#include "rdda_interfaces_types/msg/control_state.hpp"
#include "rdda_interfaces_types/msg/rdda_packet.hpp"

#include "rdda_interfaces_types/srv/homing.hpp"
#include "rdda_interfaces_types/srv/set_max_velocity.hpp"
#include "rdda_interfaces_types/srv/set_max_effort.hpp"
#include "rdda_interfaces_types/srv/set_stiffness.hpp"

/* C headers */
extern "C" {
#include "shm_data.h"
#include "shm.h"
};

class RDDNode : public rclcpp::Node {
 public:
    explicit RDDNode(const std::string& name, Rdda *rdda, const std::string& type);
    
    ~RDDNode() {}

    void run();
    void initConfigParams();
    void initCommunication();

 private:
    rclcpp::Publisher<rdda_interfaces_types::msg::RDDAPacket>::SharedPtr rdda_packet_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rdda_joint_state_pub;

    rclcpp::Subscription<rdda_interfaces_types::msg::JointCommands>::SharedPtr rdda_commands_sub;
   //  rclcpp::Subscription<rdda_interfaces_types::msg::RDDAPacket>::SharedPtr rdda_packet_sub;
   rclcpp::Subscription<rdda_interfaces_types::msg::RDDAPacket>::SharedPtr rdda_packet_sub;
   rclcpp::Subscription<rdda_interfaces_types::msg::ControlState>::SharedPtr rdda_packet_aux_pub;

    rclcpp::Service<rdda_interfaces_types::srv::SetMaxVelocity>::SharedPtr rdda_maxvel_srv;
    rclcpp::Service<rdda_interfaces_types::srv::SetMaxEffort>::SharedPtr rdda_maxeff_srv;
    rclcpp::Service<rdda_interfaces_types::srv::SetStiffness>::SharedPtr rdda_stiff_srv;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr init_srv;

    Rdda *rdda;
    std::vector<std::string> joint_names;

    std::string node_type;

    void publish_rddapacket();
    void publish_rdda_joint_state();
    void homing_finger();
   //  void rddapacket_callback(const rdda_interfaces_types::msg::RDDAPacket::SharedPtr &msg);
   
   void rddapacket_callback(const std::shared_ptr<const rdda_interfaces_types::msg::RDDAPacket>& msg);
   //  void publish_rddapacket_aux();
    void subJointCommands_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>& JointCommands_msg);

    bool setMaxVel(const std::shared_ptr<rdda_interfaces_types::srv::SetMaxVelocity::Request> req,
                   std::shared_ptr<rdda_interfaces_types::srv::SetMaxVelocity::Response> res);
    bool setMaxEffort(const std::shared_ptr<rdda_interfaces_types::srv::SetMaxEffort::Request> req,
                      std::shared_ptr<rdda_interfaces_types::srv::SetMaxEffort::Response> res);
    bool setStiffness(const std::shared_ptr<rdda_interfaces_types::srv::SetStiffness::Request> req,
                      std::shared_ptr<rdda_interfaces_types::srv::SetStiffness::Response> res);
    bool initSlave(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                   std::shared_ptr<std_srvs::srv::Empty::Response> res);

    bool teleop_connection_index;

    int delay_max = 2000*10;
    double pos_d[20000][MOTOR_COUNT];
    double vel_d[20000][MOTOR_COUNT];
    double pre_d[20000][MOTOR_COUNT];
    double wave_d[20000][MOTOR_COUNT];
    double pos_dd[20000][MOTOR_COUNT];
    int added_delay = 2000 * 3;
    int current_index = 0;
    int delay_index;
    void publish_rddapacket_delay();
};

#endif /* RDDA_NODE_H */
