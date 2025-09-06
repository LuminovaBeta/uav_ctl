#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <ros/ros.h>
#include <string>
#include <cmath>

// ROS 消息与服务
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// 强类型枚举，定义飞行状态
enum class FlightState {
    IDLE, /* 空闲 */
    ARMING,
    TAKING_OFF,
    HOVERING, /* 悬停 */
    MOVING, /* 移动 */
    LANDING /* 降落 */
};

class TrajectoryExecutorNode {
public:
    // 构造函数
    TrajectoryExecutorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    // 析构函数
    ~TrajectoryExecutorNode();

private:
    /* --- ROS 接口 --- */
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber goto_sub_;

    ros::Publisher pos_setpoint_pub_;
    ros::Publisher status_pub_;
    ros::Publisher waypoint_reached_pub_;
    bool takeoff_done_published_ = false;
    ros::Publisher takeoff_done_pub_;
    ros::ServiceServer takeoff_service_;
    ros::ServiceServer land_service_;
    ros::ServiceServer hover_service_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    ros::Timer control_timer_;

    /* --- 参数 --- */
    std::string namespace_;
    std::string log_prefix_;
    double takeoff_height_;
    double max_vel_;
    double goal_tolerance_;
    double control_rate_;

    /* --- 内部状态 --- */
    FlightState flight_state_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped hover_pose_;

    // --- 轨迹执行状态 ---
    bool trajectory_initialized_;
    ros::Time trajectory_start_time_;
    geometry_msgs::Pose trajectory_start_pose_;
    double trajectory_duration_;
    bool waypoint_task_active_;

    // --- 回调函数 ---
    void stateCb(const mavros_msgs::State::ConstPtr& msg);
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void handleGotoTopic(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool handleTakeoff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool handleLand(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool handleHover(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    // --- 主循环与逻辑函数 ---
    void controlLoop(const ros::TimerEvent& event);
    void armingLogic();
    void takeoffLogic();
    void hoverLogic();
    void movingLogic();
    void landingLogic();

    // --- 辅助函数 ---
    void setState(FlightState new_state);
    void enterHoverState();
    void enterAirIdleState();
    std::string stateToString(FlightState state);
    static double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
};

#endif // TRAJECTORY_EXECUTOR_H

