#include "rdda_interface/rdda_interface.h"

using namespace std;

bool RDDNode::setMaxVel(const std::shared_ptr<rdda_interfaces_types::srv::SetMaxVelocity::Request> req,
    std::shared_ptr<rdda_interfaces_types::srv::SetMaxVelocity::Response> res) {
    
    mutex_lock(&rdda->mutex);

    req->max_vel.resize(MOTOR_COUNT);
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].vel_sat = req->max_vel[i];
    }

    res->err = 0;
    RCLCPP_INFO(this->get_logger(), "Request: vel_sat = [%lf, %lf, %lf]", (double)req->max_vel[0], (double)req->max_vel[1], (double)req->max_vel[2]);
    RCLCPP_INFO(this->get_logger(), "Error indicator: %d", res->err);

    mutex_unlock(&rdda->mutex);
    return true;
    }

bool RDDNode::setMaxEffort(const std::shared_ptr<rdda_interfaces_types::srv::SetMaxEffort::Request> req,
          std::shared_ptr<rdda_interfaces_types::srv::SetMaxEffort::Response> res) {
    mutex_lock(&rdda->mutex);

    req->max_effort.resize(MOTOR_COUNT);

    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].tau_sat = req->max_effort[i];
    }
    res->err = 0;
    RCLCPP_INFO(this->get_logger(), "Request: tau_sat = [%lf, %lf, %lf]", (double)req->max_effort[0], (double)req->max_effort[1], (double)req->max_effort[2]);
    mutex_unlock(&rdda->mutex);
    return true;
    }

bool RDDNode::setStiffness(const std::shared_ptr<rdda_interfaces_types::srv::SetStiffness::Request> req,
          std::shared_ptr<rdda_interfaces_types::srv::SetStiffness::Response> res) {
    mutex_lock(&rdda->mutex);
    req->stiffness.resize(MOTOR_COUNT);

    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].stiffness = req->stiffness[i];
    }
    res->err = 0;

    RCLCPP_INFO(this->get_logger(), "Request: stiffness = [%lf, %lf, %lf]", (double)req->stiffness[0], (double)req->stiffness[1], (double)req->stiffness[2]);
    mutex_unlock(&rdda->mutex);
    return true;
    }


inline bool RDDNode::initSlave(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                               std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    return true;
}

/* RDDNode constructor */
RDDNode::RDDNode(const std::string& name, Rdda *rddaptr, const std::string& type)
  : Node(name), rdda(rddaptr), node_type(type) {

    if (node_type == "right_gripper") {
      this->rdda_packet_sub = this->create_subscription<rdda_interfaces_types::msg::RDDAPacket>(
          "/rdda_right_master_output", 1,
          std::bind(&RDDNode::rddapacket_callback, this, std::placeholders::_1));
      this->rdda_packet_pub = this->create_publisher<rdda_interfaces_types::msg::RDDAPacket>("/rdda_right_master_input", 1);
      this->rdda_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/right_gripper_joint_states", 1);
    }
    else if (node_type == "left_gripper") {
      this->rdda_packet_sub = this->create_subscription<rdda_interfaces_types::msg::RDDAPacket>(
          "/rdda_left_master_output", 1,
          std::bind(&RDDNode::rddapacket_callback, this, std::placeholders::_1));
      this->rdda_packet_pub = this->create_publisher<rdda_interfaces_types::msg::RDDAPacket>("/rdda_left_master_input", 1);
      this->rdda_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/left_gripper_joint_states", 1);
    }
    else if (node_type == "right_glove") {
      this->rdda_packet_sub = this->create_subscription<rdda_interfaces_types::msg::RDDAPacket>(
          "/rdda_right_slave_output", 1,
          std::bind(&RDDNode::rddapacket_callback, this, std::placeholders::_1));
      this->rdda_packet_pub = this->create_publisher<rdda_interfaces_types::msg::RDDAPacket>("/rdda_right_slave_input", 1);
      this->rdda_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/right_glove_joint_states", 1);
    }
    else if (node_type == "left_glove") {
      this->rdda_packet_sub = this->create_subscription<rdda_interfaces_types::msg::RDDAPacket>(
          "/rdda_left_slave_output", 1,
          std::bind(&RDDNode::rddapacket_callback, this, std::placeholders::_1));
      this->rdda_packet_pub = this->create_publisher<rdda_interfaces_types::msg::RDDAPacket>("/rdda_left_slave_input", 1);
      this->rdda_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/left_glove_joint_states", 1);
    }

    RCLCPP_INFO(this->get_logger(), "Node initialized");
  
    rdda_maxvel_srv = this->create_service<rdda_interfaces_types::srv::SetMaxVelocity>(
    "set_max_vel", std::bind(&RDDNode::setMaxVel, this, std::placeholders::_1, std::placeholders::_2));

    rdda_maxeff_srv = this->create_service<rdda_interfaces_types::srv::SetMaxEffort>(
        "set_max_eff", std::bind(&RDDNode::setMaxEffort, this, std::placeholders::_1, std::placeholders::_2));

    rdda_stiff_srv = this->create_service<rdda_interfaces_types::srv::SetStiffness>(
        "set_stiff", std::bind(&RDDNode::setStiffness, this, std::placeholders::_1, std::placeholders::_2));

  }

// RDDNode::~RDDNode() = default;
/* Initialize interface with ROS parameters. */
void RDDNode::initConfigParams() {
    double freq, stiff[MOTOR_COUNT], max_vel[MOTOR_COUNT], max_eff[MOTOR_COUNT];

    RCLCPP_INFO(this->get_logger(), "Node config initialized");
    teleop_connection_index = false;

    if (node_type == "right_gripper") joint_names = {"right_index_flex_motor_joint", "right_thumb_flex_motor_joint", "right_thumb_swivel_motor_joint"};
    else if (node_type == "left_gripper") joint_names = {"left_index_flex_motor_joint", "left_thumb_flex_motor_joint", "left_thumb_swivel_motor_joint"};

    // Homing slave gripper
    if (node_type == "right_gripper" || node_type == "left_gripper") {
        this->homing_finger();
        RCLCPP_INFO(this->get_logger(), "Slave gripper homed");
    }

    if (node_type == "remote") {
        this->homing_finger();
        RCLCPP_INFO(this->get_logger(), "Remote gripper homed");
    }

    rdda->ts.remote_stamp = this->get_clock()->now().seconds();

}

// Homing slave gripper finger before actuation
void RDDNode::homing_finger() {
    rdda_interfaces_types::msg::RDDAPacket packet_msg;

    double tau_upper_limit = 0.3;
    double tau_lower_limit = -0.4;
    double control_step = 0.05;
    std::vector<double> pos_ref{rdda->motor[0].rddaPacket.pos_out, rdda->motor[1].rddaPacket.pos_out, rdda->motor[2].rddaPacket.pos_out};
    rclcpp::Rate loop_rate(20);
    std::vector<bool> opened{false, false};

    // set homing stiffness to 5.0
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].rddaPacket.pos_ref = pos_ref[i];
    }

    // IMPORTANT!! For pos_ref to be updated
    rclcpp::sleep_for(std::chrono::milliseconds(100));    

    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].stiffness = 5.0;
    }

    // Open two fingers to lower bound
    while(rclcpp::ok() && (!opened[0] || !opened[1] || !opened[2])) {
        for (int i = 0; i < MOTOR_COUNT; i ++) {
            if (rdda->motor[i].rddaPacket.tau < tau_upper_limit) {
                // mutex_lock(&rdda->mutex);
                pos_ref[i] += control_step;
                rdda->motor[i].rddaPacket.pos_ref = pos_ref[i];
                std::cout << rdda->motor[0].rddaPacket.tau << " " << rdda->motor[1].rddaPacket.tau << "" 
                << rdda->motor[2].rddaPacket.tau << std::endl;
                // mutex_unlock(&rdda->mutex);
            }
            else {
                opened[i] = true;
            }
        }
        loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Fingers opened to the max");

    // Reset stiffness to 0
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].stiffness = 0.0;
        rdda->motor[i].rddaPacket.tau_ref = 0.2;
    }
    
    // Wait for stiffness to be updated
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    // Reset motor init position
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].init_pos = rdda->motor[i].motorIn.act_pos;
    }

    // init_srv = this->create_service<std_srvs::srv::Empty>(
    //         "/slave_initialized",
    //         std::bind(&RDDNode::initSlave, this, std::placeholders::_1, std::placeholders::_2));
    // RCLCPP_INFO(this->get_logger(), "Slave gripper initialized");
}

/* Publish rdda joint msgs at 100Hz */
void RDDNode::publish_rdda_joint_state() {
    sensor_msgs::msg::JointState states;
    double series_stiffness = 4.0;
    states.effort.resize(joint_names.size());
    states.name.resize(joint_names.size());
    states.position.resize(joint_names.size());
    states.velocity.resize(joint_names.size());
    states.header.stamp = this->get_clock()->now();
    mutex_lock(&rdda->mutex);
    for (size_t i = 0; i < joint_names.size(); i ++) {
        states.name[i] = joint_names[i];
        states.position[i] = rdda->motor[i].rddaPacket.pos_out + rdda->motor[i].motorIn.act_pre / series_stiffness;
        states.velocity[i] = rdda->motor[i].rddaPacket.vel_out;
        states.effort[i] = -rdda->motor[i].motorIn.act_pre;
    }
    mutex_unlock(&rdda->mutex);
    rdda_joint_state_pub->publish(states);
}

/* Publish rdda packet through ROS */
void RDDNode::publish_rddapacket() {

    rdda_interfaces_types::msg::RDDAPacket packet_msg;
    packet_msg.pos.resize(MOTOR_COUNT);
    packet_msg.vel.resize(MOTOR_COUNT);
    packet_msg.tau.resize(MOTOR_COUNT);
    packet_msg.wave.resize(MOTOR_COUNT);
    // packet_msg.wave_aux.resize(MOTOR_COUNT);
    packet_msg.pressure.resize(MOTOR_COUNT);
    // packet_msg.contact_flag.resize(MOTOR_COUNT);
    // packet_msg.test.resize(MOTOR_COUNT);
    packet_msg.delay_energy_reservior.resize(MOTOR_COUNT);
    packet_msg.pos_d.resize(MOTOR_COUNT);
    packet_msg.energy.resize(MOTOR_COUNT);
    packet_msg.ct.resize(MOTOR_COUNT);

    mutex_lock(&rdda->mutex);

    for (int i = 0; i < MOTOR_COUNT; i ++) {
        packet_msg.pos[i] = rdda->motor[i].rddaPacket.pos_out;
        packet_msg.vel[i] = rdda->motor[i].rddaPacket.vel_out;
        packet_msg.tau[i] = rdda->motor[i].rddaPacket.tau;
        // packet_msg.contact_flag[i] = rdda->motor[i].rddaPacket.contact_flag;
        packet_msg.wave[i] = rdda->motor[i].rddaPacket.wave_out;
        // packet_msg.wave_aux[i] = rdda->motor[i].rddaPacket.wave_out_aux;
        // packet_msg.test[i] = rdda->motor[i].rddaPacket.test;
        packet_msg.pressure[i] = rdda->motor[i].motorIn.act_pre;
        packet_msg.delay_energy_reservior[i] = rdda->motor[i].rddaPacket.delay_energy_reservior;
        packet_msg.pos_d[i] = rdda->motor[i].rddaPacket.pos_d_out;
        packet_msg.energy[i] = rdda->motor[i].rddaPacket.energy_tdpa_out;
        packet_msg.ct[i] = rdda->motor[i].rddaPacket.coupling_torque_out;

    }

    packet_msg.error_signal = rdda->error_signal.error_out;
    packet_msg.local_stamp = this->get_clock()->now().seconds();
    packet_msg.remote_stamp = rdda->ts.remote_stamp;
    packet_msg.time_delay = rdda->ts.delay_cycle * 0.25e-3;

    mutex_unlock(&rdda->mutex);
    rdda_packet_pub->publish(packet_msg);
}

/* Subscriber callback */
/* Comment out callback for remote test */
void RDDNode::rddapacket_callback(const std::shared_ptr<const rdda_interfaces_types::msg::RDDAPacket>& packet_msg){
    mutex_lock(&rdda->mutex);
    for (int i = 0; i < MOTOR_COUNT; i ++) {
        rdda->motor[i].rddaPacket.pos_in = packet_msg->pos[i];
        rdda->motor[i].rddaPacket.vel_in = packet_msg->vel[i];
        rdda->motor[i].rddaPacket.wave_in = packet_msg->wave[i];
        // rdda->motor[i].rddaPacket.wave_in_aux = packet_msg->wave_aux[i];
        rdda->motor[i].rddaPacket.pos_d_in = packet_msg->pos_d[i];
        // rdda->motor[i].rddaPacket.pre_in = packet_msg->pressure[i];
    }
    rdda->error_signal.error_in = packet_msg->error_signal;
    rdda->ts.remote_stamp = packet_msg->local_stamp;
    double last_local_stamp = packet_msg->remote_stamp;
    rclcpp::Time local_stamp = this->get_clock()->now();
    rdda->ts.delay_cycle = int((local_stamp.seconds() - last_local_stamp) / 0.25e-3 / 2);

    if (!teleop_connection_index) {
        for (int i = 0; i < MOTOR_COUNT; i ++) {
            rdda->motor[i].rddaPacket.tau_ref = 0.0;
        }
    }
    mutex_unlock(&rdda->mutex);

    teleop_connection_index = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Write into rdda memory, message delay: %lf", (local_stamp.seconds() - last_local_stamp) / 2);

}

void RDDNode::subJointCommands_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>& JointCommands_msg) {

        mutex_lock(&rdda->mutex);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            rdda->motor[i].rddaPacket.pos_ref = JointCommands_msg->positions[i];
        }
        mutex_unlock(&rdda->mutex);
        RCLCPP_INFO(this->get_logger(), "Set position reference: [%lf, %lf, %lf]", 
            JointCommands_msg->positions[0], 
            JointCommands_msg->positions[1], 
            JointCommands_msg->positions[2]);
    }


// /* Service functions */

// /* Run loop */
void RDDNode::run() {
    int teleop_freq = 2000;
    int joint_state_pub_freq = 10;
    int joint_state_pub_index = 0;
    rclcpp::Rate loop_rate(teleop_freq);
    while (rclcpp::ok()) {
        /* Publisher (wrap) */
        publish_rddapacket();
        if (joint_state_pub_index >= int(teleop_freq/joint_state_pub_freq)) joint_state_pub_index = 0;
        if (joint_state_pub_index == 0) {publish_rdda_joint_state();}
        joint_state_pub_index++;
        /* Subscriber callback loop */
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    // Instanciate input-output data variables
    Rdda *rdda;

    // Map data structs to shared memory
    // Open and obtain shared memory pointers for master-input data
    char *name = &argv[1][0];
    rdda = initRdda(name);
    if (rdda == nullptr) {
        fprintf(stderr, "Init rdda failed.\n");
        printf("shm_open error, errno(%d): %s\n", errno, strerror(errno));
        exit(1);
    }

    // Initialize ROS node
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rdda_interface"), "Launch ros interface");

    // Create a ROS2 node
    auto node = std::make_shared<RDDNode>(name, rdda,  std::string(argv[1]));

    // Initialize configuration parameters
    node->initConfigParams();

    rclcpp::spin(node);

    // Shutdown ROS node
    rclcpp::shutdown();

    return 0;
}

