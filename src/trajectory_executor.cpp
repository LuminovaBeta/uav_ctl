#include "uav_ctl/trajectory_executor.h"

/**
 * @file trajectory_executor.h
 * @brief 无人机飞行任务执行器节点 (Drone Flight Task Executor Node)
 * @author WSHSM (original author), C++ version by [Your Name]
 * @date 2025-09-01
 *
 * @details
 * 该节点作为无人机的中层控制器，接收高层指令并转化为连续的位置设定点。
 *
 * @par 调用逻辑 (Usage Logic)
 * 遵循“指令-等待-反馈”模式:
 * 1. 调用 `~takeoff` 服务起飞。
 * 2. 飞机悬停后，发布目标点至 `~goto` 话题以开始移动。
 * 3. 监听 `~waypoint_reached` 话题，收到 `true` 即表示到达。
 * 4. 重复步骤 2 和 3 执行多航点任务，最后调用 `~land` 服务降落。
 */

 // 构造函数，传入全局和私有 NodeHandlec 并成员初始化：初始状态 IDLE
TrajectoryExecutorNode::TrajectoryExecutorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), flight_state_(FlightState::IDLE), trajectory_initialized_(false), waypoint_task_active_(false)
{
    // --- 读取参数 ---
    pnh.param<std::string>("namespace", namespace_, "");
    pnh.param<double>("takeoff_height", takeoff_height_, 1.5);
    pnh.param<double>("max_velocity", max_vel_, 1.8);
    pnh.param<double>("goal_tolerance", goal_tolerance_, 0.2);
    pnh.param<double>("control_rate", control_rate_, 50.0);

    log_prefix_ = namespace_.empty() ? "[Executor]" : "[" + namespace_ + "]";

    ROS_INFO("%s 正在初始化轨迹执行器 v3.1（C++ 版）...", log_prefix_.c_str());

    // --- 话题/服务命名 ---
    std::string ns_prefix = namespace_.empty() ? "" : "/" + namespace_;
    std::string state_topic = ns_prefix + "/mavros/state";
    std::string pose_topic = ns_prefix + "/mavros/local_position/pose";
    std::string setpoint_topic = ns_prefix + "/mavros/setpoint_position/local";
    std::string arming_srv = ns_prefix + "/mavros/cmd/arming";
    std::string set_mode_srv = ns_prefix + "/mavros/set_mode";

    // --- 初始化 MAVROS 接口 ---
    ROS_INFO("%s 正在等待 MAVROS 服务...", log_prefix_.c_str());
    ros::service::waitForService(arming_srv);
    ros::service::waitForService(set_mode_srv);
    ROS_INFO("%s MAVROS 服务已连接。", log_prefix_.c_str());

    state_sub_ = nh_.subscribe(state_topic, 1, &TrajectoryExecutorNode::stateCb, this);
    pose_sub_ = nh_.subscribe(pose_topic, 1, &TrajectoryExecutorNode::poseCb, this);
    pos_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(setpoint_topic, 1);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_srv);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_srv);

    // --- 初始化控制接口 ---
    goto_sub_ = pnh.subscribe("goto", 1, &TrajectoryExecutorNode::handleGotoTopic, this);
    takeoff_service_ = pnh.advertiseService("takeoff", &TrajectoryExecutorNode::handleTakeoff, this);
    land_service_ = pnh.advertiseService("land", &TrajectoryExecutorNode::handleLand, this);
    hover_service_ = pnh.advertiseService("hover", &TrajectoryExecutorNode::handleHover, this);

    // --- 初始化状态/反馈发布器 ---
    status_pub_ = pnh.advertise<std_msgs::String>("status", 1, true); // Latching
    waypoint_reached_pub_ = pnh.advertise<std_msgs::Bool>("waypoint_reached", 1);
    
    // 发布初始状态
    setState(FlightState::IDLE);

    // --- 启动主循环 ---
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &TrajectoryExecutorNode::controlLoop, this);
    ROS_INFO("%s 轨迹执行器已就绪。", log_prefix_.c_str());
}

TrajectoryExecutorNode::~TrajectoryExecutorNode() {}

void TrajectoryExecutorNode::stateCb(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void TrajectoryExecutorNode::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}

void TrajectoryExecutorNode::handleGotoTopic(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if ((flight_state_ == FlightState::HOVERING || flight_state_ == FlightState::IDLE) && current_state_.armed) {
        ROS_INFO("收到 GoTo 指令，目标：[%.2f, %.2f, %.2f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        target_pose_ = *msg;
        trajectory_initialized_ = false;
        waypoint_task_active_ = true; // 标记开始执行 GOTO 任务
        setState(FlightState::MOVING);
    } else {
        ROS_WARN("无法执行 GoTo。当前状态：%s（解锁状态: %d）", stateToString(flight_state_).c_str(), current_state_.armed);
    }
}

bool TrajectoryExecutorNode::handleTakeoff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (flight_state_ == FlightState::IDLE && !current_state_.armed) {
        ROS_INFO("收到起飞指令，开始执行...");
        setState(FlightState::ARMING);
        res.success = true;
        res.message = "已开始执行起飞流程。";
    } else {
        res.success = false;
        res.message = "无法起飞，当前状态为 " + stateToString(flight_state_);
    }
    return true;
}

bool TrajectoryExecutorNode::handleLand(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if ((flight_state_ == FlightState::HOVERING || flight_state_ == FlightState::MOVING || flight_state_ == FlightState::IDLE) && current_state_.armed) {
        ROS_INFO("收到降落指令，正在降落...");
        setState(FlightState::LANDING);
        res.success = true;
        res.message = "已开始执行降落流程。";
    } else {
        res.success = false;
        res.message = "无法降落，当前状态为 " + stateToString(flight_state_);
    }
    return true;
}

bool TrajectoryExecutorNode::handleHover(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (flight_state_ == FlightState::MOVING) {
        ROS_INFO("收到悬停指令，正在中断移动并悬停。");
        enterHoverState();
        res.success = true;
        res.message = "悬停指令已接受。";
    } else {
        res.success = false;
        res.message = "无法悬停，当前状态为 " + stateToString(flight_state_);
    }
    return true;
}

void TrajectoryExecutorNode::controlLoop(const ros::TimerEvent& event) {
    switch (flight_state_) {
        case FlightState::ARMING:       armingLogic();      break;
        case FlightState::TAKING_OFF:   takeoffLogic();     break;
        case FlightState::HOVERING:     hoverLogic();       break;
        case FlightState::MOVING:       movingLogic();      break;
        case FlightState::LANDING:      landingLogic();     break;
        case FlightState::IDLE:
            if (current_state_.armed) {
                // 空中 IDLE 时保持当前位置悬停
                pos_setpoint_pub_.publish(current_pose_);
            }
            break;
    }
}

void TrajectoryExecutorNode::armingLogic() {
    pos_setpoint_pub_.publish(current_pose_); // 切换 OFFBOARD 前持续发送设定点
    if (current_state_.mode != "OFFBOARD") {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client_.call(offb_set_mode);
    } else if (!current_state_.armed) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        arming_client_.call(arm_cmd);
    } else {
        ROS_INFO("已进入 OFFBOARD 模式并完成解锁，准备起飞。");
        setState(FlightState::TAKING_OFF);
    }
}

void TrajectoryExecutorNode::takeoffLogic() {
    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map"; // 或者使用你的参考坐标系
    target.pose.position.x = current_pose_.pose.position.x;
    target.pose.position.y = current_pose_.pose.position.y;
    target.pose.position.z = takeoff_height_;
    target.pose.orientation = current_pose_.pose.orientation;
    pos_setpoint_pub_.publish(target);

    if (std::abs(current_pose_.pose.position.z - takeoff_height_) < goal_tolerance_) {
        ROS_INFO("起飞完成，进入悬停状态。");
        enterHoverState();
    }
}

void TrajectoryExecutorNode::hoverLogic() {
    hover_pose_.header.stamp = ros::Time::now();
    pos_setpoint_pub_.publish(hover_pose_);
}

void TrajectoryExecutorNode::movingLogic() {
    if (!trajectory_initialized_) {
        trajectory_start_time_ = ros::Time::now();
        trajectory_start_pose_ = current_pose_.pose;
        double dist = distance(trajectory_start_pose_.position, target_pose_.pose.position);
        double effective_vel = std::max(max_vel_, 0.01);
        trajectory_duration_ = dist / effective_vel;
        trajectory_initialized_ = true;
        ROS_INFO("开始新的轨迹：距离=%.2f m，预计时长=%.2f s", dist, trajectory_duration_);
    }

    double elapsed = (ros::Time::now() - trajectory_start_time_).toSec();
    double s = std::max(0.0, std::min(1.0, trajectory_duration_ > 1e-6 ? elapsed / trajectory_duration_ : 1.0));

    const auto& start_pos = trajectory_start_pose_.position;
    const auto& goal_pos = target_pose_.pose.position;

    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.pose.position.x = start_pos.x + s * (goal_pos.x - start_pos.x);
    setpoint.pose.position.y = start_pos.y + s * (goal_pos.y - start_pos.y);
    setpoint.pose.position.z = start_pos.z + s * (goal_pos.z - start_pos.z);
    setpoint.pose.orientation = target_pose_.pose.orientation;
    pos_setpoint_pub_.publish(setpoint);

    if (distance(current_pose_.pose.position, goal_pos) < goal_tolerance_) {
        ROS_INFO("到达目标航点。");
        if (waypoint_task_active_) {
            std_msgs::Bool reached_msg;
            reached_msg.data = true;
            waypoint_reached_pub_.publish(reached_msg);
            ROS_INFO("已发布 waypoint_reached 信号：True");
            waypoint_task_active_ = false;
        }
        enterAirIdleState();
    }
}

void TrajectoryExecutorNode::landingLogic() {
    if (current_state_.mode != "AUTO.LAND") {
        mavros_msgs::SetMode land_mode;
        land_mode.request.custom_mode = "AUTO.LAND";
        set_mode_client_.call(land_mode);
    }
    if (!current_state_.armed) {
        ROS_INFO("飞机已落地并加锁，返回地面 IDLE。");
        setState(FlightState::IDLE);
    }
}

void TrajectoryExecutorNode::setState(FlightState new_state) {
    if (flight_state_ != new_state) {
        flight_state_ = new_state;
        std_msgs::String status_msg;
        status_msg.data = stateToString(new_state);
        status_pub_.publish(status_msg);
        ROS_INFO("状态切换 -> %s", status_msg.data.c_str());
    }
}

void TrajectoryExecutorNode::enterHoverState() {
    hover_pose_ = current_pose_;
    setState(FlightState::HOVERING);
}

void TrajectoryExecutorNode::enterAirIdleState() {
    setState(FlightState::IDLE);
}

std::string TrajectoryExecutorNode::stateToString(FlightState state) {
    switch (state) {
        case FlightState::IDLE:         return "IDLE";
        case FlightState::ARMING:       return "ARMING";
        case FlightState::TAKING_OFF:   return "TAKING_OFF";
        case FlightState::HOVERING:     return "HOVERING";
        case FlightState::MOVING:       return "MOVING";
        case FlightState::LANDING:      return "LANDING";
        default:                        return "UNKNOWN";
    }
}

double TrajectoryExecutorNode::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// --- Main Function ---
int main(int argc, char **argv) {
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "trajectory_executor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // 私有命名空间参数
    
    try {
        TrajectoryExecutorNode executor(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("trajectory_executor 异常: %s", e.what());
    }
    
    ROS_INFO("节点退出。");
    return 0;
}
