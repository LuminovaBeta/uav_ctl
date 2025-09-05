#include <ros/ros.h>
#include <vector>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpcValue.h>

// =====================================================================================
// 结构体和枚举定义 (Definitions)
// =====================================================================================

// 主规划器状态
enum class PlannerState {
    IDLE,
    TAKEOFF,
    MISSION,
    LANDING,
    FINISHED
};

// =====================================================================================
// 主规划器类定义 (Planner Class Definition) - 已简化为单机
// =====================================================================================

class MissionPlannerNode {
public:
    MissionPlannerNode(ros::NodeHandle& nh);

private:
    void loadWaypoints();
    bool startMissionCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void reachedCb(const std_msgs::Bool::ConstPtr& msg);
    void update(const ros::TimerEvent& event);

    // ROS 接口
    ros::NodeHandle nh_;
    ros::ServiceServer start_mission_service_;
    ros::Timer update_timer_;
    ros::Publisher goto_pub_;
    ros::Subscriber reached_sub_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;

    // 任务数据
    std::string uav_ns_;
    std::vector<geometry_msgs::Point> waypoints_;
    bool task_complete_ = false; // 同步标志位
    
    // 状态机变量
    int current_wp_index_;
    PlannerState state_;
    ros::Time phase_start_time_; // 用于任务超时
};

// =====================================================================================
// 类成员函数的实现 (Class Method Implementations)
// =====================================================================================

MissionPlannerNode::MissionPlannerNode(ros::NodeHandle& nh) : nh_(nh), current_wp_index_(-1), state_(PlannerState::IDLE) {
    ros::NodeHandle pnh("~");

    // 获取无人机的命名空间 (如果需要)
    pnh.param<std::string>("uav_ns", uav_ns_, "uav");

    loadWaypoints();

    // 初始化无人机的ROS接口
    std::string topic_prefix = "/" + uav_ns_ + "/trajectory_executor";
    goto_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/goto", 1);
    reached_sub_ = nh_.subscribe(topic_prefix + "/waypoint_reached", 1, &MissionPlannerNode::reachedCb, this);
    takeoff_client_ = nh_.serviceClient<std_srvs::Trigger>(topic_prefix + "/takeoff");
    land_client_ = nh_.serviceClient<std_srvs::Trigger>(topic_prefix + "/land");

    start_mission_service_ = pnh.advertiseService("start_mission", &MissionPlannerNode::startMissionCb, this);
    update_timer_ = nh_.createTimer(ros::Duration(0.1), &MissionPlannerNode::update, this);

    ROS_INFO("Single UAV Mission Planner is ready. Call ~start_mission service to begin.");
}

void MissionPlannerNode::loadWaypoints() {
    ros::NodeHandle pnh("~");
    XmlRpc::XmlRpcValue waypoints_xml;
    if (!pnh.getParam("waypoints", waypoints_xml)) {
        ROS_ERROR("Failed to get 'waypoints' parameter.");
        ros::shutdown();
        return;
    }

    // 简化后的航点解析逻辑
    for (int i = 0; i < waypoints_xml.size(); ++i) {
        if (waypoints_xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray && waypoints_xml[i].size() == 3) {
            geometry_msgs::Point pt;
            pt.x = static_cast<double>(waypoints_xml[i][0]);
            pt.y = static_cast<double>(waypoints_xml[i][1]);
            pt.z = static_cast<double>(waypoints_xml[i][2]);
            waypoints_.push_back(pt);
        }
    }

    if (waypoints_.empty()) {
        ROS_ERROR("Waypoint loading error: No waypoints found or format is incorrect.");
        ros::shutdown();
    } else {
        ROS_INFO("Loaded %ld waypoints for the mission.", waypoints_.size());
    }
}

bool MissionPlannerNode::startMissionCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (state_ != PlannerState::IDLE) {
        res.success = false;
        res.message = "Planner is not IDLE.";
        return true;
    }
    ROS_INFO("Start mission command received. Starting takeoff phase.");
    state_ = PlannerState::TAKEOFF;
    phase_start_time_ = ros::Time::now();
    
    // 发送起飞指令
    std_srvs::Trigger srv;
    takeoff_client_.call(srv);

    res.success = true;
    res.message = "Mission started.";
    return true;
}

void MissionPlannerNode::reachedCb(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        task_complete_ = true;
    }
}

void MissionPlannerNode::update(const ros::TimerEvent& event) {
    // 检查超时
    if (state_ != PlannerState::IDLE && state_ != PlannerState::FINISHED) {
        if (ros::Time::now() - phase_start_time_ > ros::Duration(30.0)) { // 30秒超时
            ROS_ERROR("Mission phase timeout! Commanding LAND.");
            std_srvs::Trigger srv;
            land_client_.call(srv);
            state_ = PlannerState::FINISHED;
        }
    }

    switch (state_) {
        case PlannerState::TAKEOFF: {
            if (task_complete_) {
                ROS_INFO("UAV stabilized after takeoff. Starting mission waypoints.");
                state_ = PlannerState::MISSION;
                phase_start_time_ = ros::Time::now();
                current_wp_index_ = 0;
                task_complete_ = false;

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.orientation.w = 1.0;
                pose.header.stamp = ros::Time::now();
                pose.pose.position = waypoints_[current_wp_index_];
                goto_pub_.publish(pose);
            }
            break;
        }
        case PlannerState::MISSION: {
            if (task_complete_) {
                ROS_INFO("Reached waypoint %d.", current_wp_index_ + 1);
                current_wp_index_++;
                
                if (current_wp_index_ < waypoints_.size()) {
                    ROS_INFO("Proceeding to waypoint %d.", current_wp_index_ + 1);
                    phase_start_time_ = ros::Time::now();
                    task_complete_ = false;

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.pose.orientation.w = 1.0;
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position = waypoints_[current_wp_index_];
                    goto_pub_.publish(pose);

                } else {
                    ROS_INFO("All waypoints reached. Commanding LAND.");
                    state_ = PlannerState::LANDING;
                    phase_start_time_ = ros::Time::now();
                    std_srvs::Trigger srv;
                    land_client_.call(srv);
                }
            }
            break;
        }
        case PlannerState::LANDING: {
            if (ros::Time::now() - phase_start_time_ > ros::Duration(5.0)) {
                ROS_INFO("Mission finished.");
                state_ = PlannerState::FINISHED;
                ros::shutdown();
            }
            break;
        }
        default:
            break;
    }
}

// =====================================================================================
// 主函数 (Main Function)
// =====================================================================================

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_planner_node");
    ros::NodeHandle nh;
    MissionPlannerNode node(nh);
    ros::spin();
    return 0;
}