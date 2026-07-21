#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <decision_pkg/PlanningStatus.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <vector>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class DecisionNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher status_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;

    std::vector<geometry_msgs::PoseStamped> goals_;
    size_t current_goal_index_;
    bool goal_sent_;
    bool initial_action_completed_;
    
    bool enable_initial_action_;
    bool wait_for_tf_before_initial_action_;
    double initial_lateral_distance_;
    double initial_backward_distance_;
    double initial_move_speed_;

    geometry_msgs::Twist current_vel_;
    std::string nav_status_;
    std::string move_base_state_;
    bool goal_reached_;
    double odom_yaw_;
    double odom_yaw_deg_;
    double odom_x_;
    double odom_y_;
    bool odom_ready_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_ptr_;
    geometry_msgs::Pose camera_rel_body_pose_;
    
    // 机械臂控制状态
    uint8_t grip_run_;     // 爪子: 0关闭/1打开
    uint8_t lift_run_;     // 升降: 0下降/1上升
    uint8_t rotate_run_;   // 旋转: 0停止/1旋转
    
    double start_x_;
    double start_y_;
    
    bool tf_map_ready_;
    bool tf_body_ready_;
    ros::Time last_tf_update_;

    MoveBaseClient ac_;

public:
    DecisionNode() : current_goal_index_(0), goal_sent_(false),
                     nav_status_("ONE"), move_base_state_("IDLE"), goal_reached_(false),
                     grip_run_(1), lift_run_(0), rotate_run_(0),
                     odom_yaw_(0.0), odom_x_(0.0), odom_y_(0.0), odom_ready_(false),
                     initial_action_completed_(false),
                     tf_map_ready_(false), tf_body_ready_(false),
                     ac_("move_base_navigation/move_base", true)
    {
        ROS_INFO("==============================================");
        ROS_INFO("[DEBUG] === DecisionNode 构造函数开始 ===");
        ROS_INFO("==============================================");
        
        tf_listener_ptr_ = new tf2_ros::TransformListener(tf_buffer_);
        ROS_INFO("[DEBUG] TF监听器初始化完成");
        
        nh_.param<bool>("enable_initial_action", enable_initial_action_, true);
        nh_.param<bool>("wait_for_tf_before_initial_action", wait_for_tf_before_initial_action_, false);
        nh_.param<double>("initial_lateral_distance", initial_lateral_distance_, -0.25);
        nh_.param<double>("initial_backward_distance", initial_backward_distance_, 0.6);
        nh_.param<double>("initial_move_speed", initial_move_speed_, 0.2);
        ROS_INFO("[DEBUG] 参数加载完成");
        
        ROS_INFO("初始动作配置: 横移=%.2fm, 后退=%.2fm, 速度=%.2fm/s, 等待TF=%s", 
                 initial_lateral_distance_, initial_backward_distance_, initial_move_speed_,
                 wait_for_tf_before_initial_action_ ? "是" : "否");

        status_pub_ = nh_.advertise<decision_pkg::PlanningStatus>("/planning_status", 10);
        ROS_INFO("[DEBUG] /planning_status 发布者初始化完成，消息类型: decision_pkg/PlanningStatus");
        
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("[DEBUG] /cmd_vel 发布者初始化完成");
        
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &DecisionNode::cmdVelCallback, this);
        
        std::string odom_topic;
        nh_.param<std::string>("odom_topic", odom_topic, "/Odometry");  // 修改默认值为 /Odometry
        odom_sub_ = nh_.subscribe(odom_topic, 10, &DecisionNode::odomCallback, this);
        ROS_INFO("[DEBUG] 订阅里程计话题: %s", odom_topic.c_str());
        
        ROS_INFO("[DEBUG] 订阅者初始化完成");

        loadGoals();
        ROS_INFO("[DEBUG] 目标点加载完成");

        ROS_INFO("[DEBUG] Waiting for the move_base action server at: move_base_navigation/move_base");
        if (!ac_.waitForServer(ros::Duration(5.0))) {
            ROS_WARN("[WARN] move_base 尚未就绪，稍后再试...");
            ROS_WARN("[WARN] 初始动作仍会执行，但后续导航将等待 move_base 就绪");
            // 不退出，继续执行初始动作
        } else {
            ROS_INFO("[DEBUG] Connected to move_base action server.");
        }
        
        waitForTFReady();
        ROS_INFO("[DEBUG] TF等待完成");
        
        if (enable_initial_action_) {
            ROS_INFO("[DEBUG] enable_initial_action_=true,即将执行初始动作...");
            executeInitialActionSequence();
            ROS_INFO("[DEBUG] 初始动作执行完毕，进入主循环");
        } else {
            initial_action_completed_ = true;
            ROS_INFO("[DEBUG] enable_initial_action_=false,跳过初始动作");
        }
        
        ROS_INFO("[DEBUG] 构造函数完成");
    }

    ~DecisionNode()
    {
        delete tf_listener_ptr_;
    }

    void waitForTFReady() {
        ROS_INFO("等待TF树就绪...");
        ros::Rate rate(10);
        int timeout_count = 0;
        const int max_timeout = 50;
        
        while (ros::ok() && timeout_count < max_timeout) {
            if (checkTransform("map", "body")) {
                tf_map_ready_ = true;
            }
            if (checkTransform("body", "camera_init")) {
                tf_body_ready_ = true;
            }
            
            if (tf_map_ready_ && tf_body_ready_) {
                ROS_INFO("TF树就绪");
                return;
            }
            
            timeout_count++;
            rate.sleep();
            ros::spinOnce();
        }
        
        ROS_WARN("TF树等待超时,部分变换可能不可用");
    }

    bool checkTransform(const std::string& target_frame, const std::string& source_frame) {
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.1));
            return true;
        } catch (tf2::TransformException &ex) {
            ROS_DEBUG_THROTTLE(1.0, "TF不可用: %s -> %s: %s", 
                              target_frame.c_str(), source_frame.c_str(), ex.what());
            return false;
        }
    }

    bool getCurrentPositionNonBlocking(double& x, double& y) {
        if (tf_map_ready_) {
            try {
                geometry_msgs::TransformStamped transform = 
                    tf_buffer_.lookupTransform("map", "body", ros::Time(0), ros::Duration(0.1));
                x = transform.transform.translation.x;
                y = transform.transform.translation.y;
                last_tf_update_ = ros::Time::now();
                ROS_INFO_THROTTLE(1.0, "[DEBUG] 使用TF获取位置: (%.4f, %.4f)", x, y);
                return true;
            } catch (tf2::TransformException &ex) {
                ROS_DEBUG_THROTTLE(0.5, "TF获取位置失败: %s", ex.what());
            }
        }
        
        if (odom_ready_) {
            x = odom_x_;
            y = odom_y_;
            ROS_INFO_THROTTLE(1.0, "[DEBUG] 使用里程计获取位置: (%.4f, %.4f)", x, y);
            return true;
        }
        
        ROS_WARN_THROTTLE(1.0, "无法获取位置:TF和里程计都不可用");
        return false;
    }

    void sleepWithSpin(double duration) {
        ros::Rate rate(50);
        int iterations = static_cast<int>(duration * 50);
        for (int i = 0; i < iterations && ros::ok(); ++i) {
            publishPlanningStatus();  // 持续发布状态，确保驱动收到心跳
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void executeInitialActionSequence() {
        nav_status_ = "ONE";
        ROS_INFO("开始执行初始动作序列...");
        
        if (wait_for_tf_before_initial_action_) {
            ROS_INFO("等待TF map->body就绪...");
            ros::Rate rate(10);
            int timeout_count = 0;
            const int max_timeout = 150;
            
            while (ros::ok() && timeout_count < max_timeout && !tf_map_ready_) {
                if (checkTransform("map", "body")) {
                    tf_map_ready_ = true;
                    ROS_INFO("TF map->body就绪");
                    break;
                }
                timeout_count++;
                rate.sleep();
                ros::spinOnce();
            }
            
            if (!tf_map_ready_) {
                ROS_WARN("等待TF超时,将使用里程计数据执行初始动作");
            }
        }
        
        if (!waitForSensorReady()) {
            ROS_WARN("传感器数据未就绪，使用默认起始位置执行初始动作");
            start_x_ = 0.0;
            start_y_ = 0.0;
        } else {
            if (!getCurrentPositionNonBlocking(start_x_, start_y_)) {
                ROS_WARN("获取起始位置失败，使用默认起始位置");
                start_x_ = 0.0;
                start_y_ = 0.0;
            }
        }
        ROS_INFO("起始位置: (%.3f, %.3f)", start_x_, start_y_);
        
        grip_run_ = 1;
        lift_run_ = 0;
        rotate_run_ = 0;
        ROS_INFO("初始动作开始前：爪子张开，升降下降，旋转停止");
        // publishPlanningStatus();
        
        if (fabs(initial_lateral_distance_) > 0.01) {
            executePureLateralMovement(initial_lateral_distance_);
        }
        
        if (initial_backward_distance_ > 0.01) {
            executePureBackwardMovement(initial_backward_distance_);
        }
        
        ROS_INFO("后退完成,等待1秒后关闭爪子...");
        sleepWithSpin(1.0);
        
         grip_run_ = 0;
        ROS_INFO("爪子已关闭");
        publishPlanningStatus();
        
        ROS_INFO("等待1秒后开始上升...");
        sleepWithSpin(1.0);
        
        lift_run_ = 1;
        ROS_INFO("开始上升");
        publishPlanningStatus();
        
        ROS_INFO("上升2秒后开始向前移动...");
        sleepWithSpin(2.0);
        
        ROS_INFO("开始向前移动1米");
        ROS_INFO("[DEBUG] 即将调用 executePureForwardMovement(1.0)");
        try {
            executePureForwardMovement(1.0);
            ROS_INFO("[DEBUG] executePureForwardMovement(1.0) 已返回");
        } catch (const std::exception& e) {
            ROS_ERROR("[ERROR] executePureForwardMovement 异常: %s", e.what());
        } catch (...) {
            ROS_ERROR("[ERROR] executePureForwardMovement 发生未知异常");
        }
        ROS_INFO("[DEBUG] 向前移动完成，准备执行下降和旋转");
        
        ROS_INFO("[DEBUG] 前进完成，准备执行下降动作");
        executeLiftDown();
        
        ROS_INFO("[DEBUG] 准备执行旋转动作");
        executeRotate();
        
        initial_action_completed_ = true;
        nav_status_ = "ONE";
        ROS_INFO("初始动作序列执行完成");
        
        ROS_INFO("等待5分钟后继续导航...");
        sleepWithSpin(300.0);
    }

    bool waitForSensorReady() {
        ROS_INFO("[DEBUG] 等待传感器数据就绪...");
        ros::Rate rate(20);
        int ready_count = 0;
        const int required_ready = 20;
        int timeout_count = 0;
        const int max_timeout = 100;  // 最多等待5秒
        
        while (ros::ok() && ready_count < required_ready && timeout_count < max_timeout) {
            if (odom_ready_) {
                ready_count++;
            } else {
                ready_count = 0;
                ROS_DEBUG_THROTTLE(1.0, "[DEBUG] 等待里程计数据...");
            }
            
            timeout_count++;
            rate.sleep();
            ros::spinOnce();
        }
        
        if (timeout_count >= max_timeout) {
            ROS_WARN("[WARN] 传感器等待超时！检查里程计话题 /odom");
        }
        
        if (ready_count >= required_ready) {
            ROS_INFO("[DEBUG] 里程计数据就绪");
            return true;
        }
        ROS_WARN("[WARN] 传感器数据未就绪");
        return false;
    }

    void executePureLateralMovement(double distance) {
        ROS_INFO("=== 开始纯横向移动 ===");
        
        // 等待订阅者连接
        // 等待订阅者连接，最多12秒
        const int MAX_WAIT = 120;  // 12秒 (120 * 0.1秒)
        int wait_count = 0;
            while (status_pub_.getNumSubscribers() == 0 && ros::ok() && wait_count < MAX_WAIT) {
            if (wait_count % 10 == 0) {
            ROS_INFO("等待 /planning_status 订阅者... (%d/%d)", wait_count, MAX_WAIT);
            }
            wait_count++;
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            }

            if (status_pub_.getNumSubscribers() == 0) {
            ROS_WARN("[WARN] /planning_status 没有订阅者（等待%.1f秒超时），将继续执行", MAX_WAIT * 0.1);
        }
        
        ROS_INFO("开始纯横向移动 %.2f 米...", distance);
        
        double current_y = start_y_;
        double target_y = start_y_ + distance;
        int retry_count = 0;
        const int max_retries = 5;
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        // 硬件执行方向与命令相反，需要反转
        cmd.linear.y = -initial_move_speed_ * (distance > 0 ? 1 : -1);
        cmd.angular.z = 0.0;
        
        ROS_INFO("[DEBUG] 横向移动速度: linear.y=%.2f, 目标距离: %.2f米", cmd.linear.y, distance);
        
        ros::Rate rate(50);
        while (ros::ok()) {
            double x, y;
            
            if (!getCurrentPositionNonBlocking(x, y)) {
                retry_count++;
                if (retry_count >= max_retries) {
                    ROS_WARN("连续%d次位置查询失败,停止横向移动", max_retries);
                    break;
                }
                rate.sleep();
                ros::spinOnce();
                continue;
            }
            retry_count = 0;
            current_y = y;
            
            double diff = fabs(current_y - target_y);
            ROS_INFO_THROTTLE(0.2, "[DEBUG] Lateral move: current_y=%.3f, target_y=%.3f, diff=%.3f", current_y, target_y, diff);
            
            if (diff < 0.02) {
                ROS_INFO("Lateral move completed: y=%.3f", current_y);
                break;
            }
            
            if (diff < 0.08) {
                // Hardware direction is inverted
                cmd.linear.y = -initial_move_speed_ * (distance > 0 ? 1 : -1) / 2;
                ROS_INFO_THROTTLE(0.5, "[INFO] Approaching target (<0.08m), decelerating to: linear.y=%.2f", cmd.linear.y);
            } else {
                // Hardware direction is inverted
                cmd.linear.y = -initial_move_speed_ * (distance > 0 ? 1 : -1);
            }
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            
            if (cmd_vel_pub_.getNumSubscribers() == 0) {
                ROS_WARN_THROTTLE(1.0, "[WARN] /cmd_vel 没有订阅者！");
            }
            cmd_vel_pub_.publish(cmd);
            current_vel_ = cmd;  // 更新 current_vel_
            publishPlanningStatus();  // 立即发布状态到 /planning_status
            ROS_INFO_THROTTLE(0.5, "[INFO] 发布横向速度命令: linear.y=%.2f, 订阅者数量=%d", cmd.linear.y, cmd_vel_pub_.getNumSubscribers());
            
            ros::spinOnce();
            rate.sleep();
        }
        
        stopAllMotion();
        ros::Duration(0.3).sleep();
        getCurrentPositionNonBlocking(start_x_, start_y_);
    }

    void executePureBackwardMovement(double distance) {
        ROS_INFO("=== 开始纯后退移动 ===");
        
        // 等待订阅者连接，最多12秒
        const int MAX_WAIT = 120;  // 12秒 (120 * 0.1秒)
        int wait_count = 0;
        while (status_pub_.getNumSubscribers() == 0 && ros::ok() && wait_count < MAX_WAIT) {
            if (wait_count % 10 == 0) {
                ROS_INFO("等待 /planning_status 订阅者... (%d/%d)", wait_count, MAX_WAIT);
            }
            wait_count++;
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        if (status_pub_.getNumSubscribers() == 0) {
            ROS_WARN("[WARN] /planning_status 没有订阅者（等待%.1f秒超时），将继续执行", MAX_WAIT * 0.1);
        }
        
        ROS_INFO("开始纯后退移动 %.2f 米...", distance);
        
        double current_x, current_y;
        if (!getCurrentPositionNonBlocking(current_x, current_y)) {
            ROS_WARN("获取当前位置失败，使用上次记录的位置");
            current_x = start_x_;
        }
        ROS_INFO("[DEBUG] 初始位置: current_x=%.3f, start_x_=%.3f, 移动距离=%.2f", current_x, start_x_, distance);
        double target_x = current_x - distance;
        ROS_INFO("[DEBUG] 目标位置: target_x=%.3f", target_x);
        int retry_count = 0;
        const int max_retries = 5;
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = -initial_move_speed_;  // 负值表示后退
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        
        ros::Time start_time = ros::Time::now();
        const double max_duration = distance / initial_move_speed_ + 5.0;  // 预计时间 + 5秒缓冲
        ros::Time last_position_update = ros::Time::now();
        ros::Rate rate(50);
        bool completed_normally = false;
        
        // 添加一个最大尝试次数，防止无限循环
        int max_loop_iterations = static_cast<int>(max_duration * 50); // 50Hz * duration
        int iteration_count = 0;
        
        while (ros::ok() && iteration_count < max_loop_iterations) {
            double x, y;
            
            if (!getCurrentPositionNonBlocking(x, y)) {
                retry_count++;
                if (retry_count >= max_retries) {
                    ROS_WARN("连续%d次获取位置失败,退出循环", max_retries);
                    break;
                }
                rate.sleep();
                ros::spinOnce();
                iteration_count++;
                continue;
            }
            retry_count = 0;
            current_x = x;
            last_position_update = ros::Time::now();  // 更新最后位置获取时间
            
            double diff = fabs(current_x - target_x);
            ROS_INFO_THROTTLE(0.2, "[DEBUG] Backward move: current_x=%.3f, target_x=%.3f, diff=%.3f", current_x, target_x, diff);
            
            if (diff < 0.02) {
                ROS_INFO("Backward move completed: x=%.3f", current_x);
                completed_normally = true;
                break;
            }
            
            if (diff < 0.08) {
                cmd.linear.x = -initial_move_speed_ / 2;  // 减速
                ROS_INFO_THROTTLE(0.5, "[INFO] Approaching target (<0.08m), decelerating to: linear.x=%.2f", cmd.linear.x);
            } else {
                cmd.linear.x = -initial_move_speed_;  // 正常速度
            }
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            
            cmd_vel_pub_.publish(cmd);
            current_vel_ = cmd;  // 更新 current_vel_
            publishPlanningStatus();  // 立即发布状态到 /planning_status
            ROS_INFO_THROTTLE(0.5, "[INFO] 发布后退速度命令: linear.x=%.2f", cmd.linear.x);
            
            ros::spinOnce();
            rate.sleep();
            iteration_count++;
            
            // Check if we haven't received a new position update in a while
            if ((ros::Time::now() - last_position_update).toSec() > 5.0) {  // 5秒无位置更新
                ROS_WARN("超过5秒未收到位置更新,强制退出后退移动");
                break;
            }
            
            if ((ros::Time::now() - start_time).toSec() > max_duration) {
                ROS_WARN("后退移动超时（%.2f秒），强制退出", max_duration);
                break;
            }
        }
        
        if (iteration_count >= max_loop_iterations) {
            ROS_WARN("后退移动达到最大迭代次数限制，强制退出");
        }
        
        ROS_INFO("[DEBUG] executePureBackwardMovement() 循环结束,completed_normally=%s,准备停止运动", completed_normally ? "true" : "false");
        stopAllMotion();
        ROS_INFO("[DEBUG] stopAllMotion() 完成");
        ros::Duration(0.3).sleep();
        ROS_INFO("[DEBUG] executePureBackwardMovement() 即将返回");
    }

    void executePureForwardMovement(double distance) {
        ROS_INFO("=== 开始纯向前移动 ===");
        
        // 等待订阅者连接，最多12秒
        const int MAX_WAIT = 120;  // 12秒 (120 * 0.1秒)
        int wait_count = 0;
        while (status_pub_.getNumSubscribers() == 0 && ros::ok() && wait_count < MAX_WAIT) {
            if (wait_count % 10 == 0) {
                ROS_INFO("等待 /planning_status 订阅者... (%d/%d)", wait_count, MAX_WAIT);
            }
            wait_count++;
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        if (status_pub_.getNumSubscribers() == 0) {
            ROS_WARN("[WARN] /planning_status 没有订阅者（等待%.1f秒超时），将继续执行", MAX_WAIT * 0.1);
        }
        
        ROS_INFO("开始纯向前移动 %.2f 米...", distance);
        
        double current_x, current_y;
        if (!getCurrentPositionNonBlocking(current_x, current_y)) {
            ROS_WARN("获取当前位置失败，使用上次记录的位置");
            current_x = start_x_;
        }
        ROS_INFO("[DEBUG] 初始位置: current_x=%.3f, start_x_=%.3f, 移动距离=%.2f", current_x, start_x_, distance);
        double target_x = current_x + distance;
        ROS_INFO("[DEBUG] 目标位置: target_x=%.3f", target_x);
        int retry_count = 0;
        const int max_retries = 5;
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = initial_move_speed_;  // 正向移动
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        
        ros::Rate rate(50);
        ros::Time start_time = ros::Time::now();
        const double max_duration = distance / initial_move_speed_ + 5.0;  // 预计时间 + 5秒缓冲
        
        bool completed_normally = false;
        while (ros::ok()) {
            double x, y;
            
            if (!getCurrentPositionNonBlocking(x, y)) {
                retry_count++;
                if (retry_count >= max_retries) {
                    ROS_WARN("连续%d次获取位置失败,退出循环", max_retries);
                    break;
                }
                rate.sleep();
                ros::spinOnce();
                continue;
            }
            retry_count = 0;
            current_x = x;
            
            double diff = fabs(current_x - target_x);
            ROS_INFO_THROTTLE(0.2, "[DEBUG] Forward move: current_x=%.3f, target_x=%.3f, diff=%.3f", current_x, target_x, diff);
            
            if (diff < 0.02) {
                ROS_INFO("Forward move completed: x=%.3f", current_x);
                completed_normally = true;
                break;
            }
            
            if (diff < 0.08) {
                cmd.linear.x = initial_move_speed_ / 2;  // 减速
                ROS_INFO_THROTTLE(0.5, "[INFO] Approaching target (<0.08m), decelerating to: linear.x=%.2f", cmd.linear.x);
            } else {
                cmd.linear.x = initial_move_speed_;  // 正常速度
            }
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            
            cmd_vel_pub_.publish(cmd);
            current_vel_ = cmd;  // 更新 current_vel_
            publishPlanningStatus();  // 立即发布状态到 /planning_status
            ROS_INFO_THROTTLE(0.5, "[INFO] 发布向前速度命令: linear.x=%.2f", cmd.linear.x);
            
            ros::spinOnce();
            rate.sleep();
            
            if ((ros::Time::now() - start_time).toSec() > max_duration) {
                ROS_WARN("向前移动超时（%.2f秒），强制退出", max_duration);
                break;
            }
        }
        
        ROS_INFO("[DEBUG] executePureForwardMovement() 循环结束,completed_normally=%s,准备停止运动", completed_normally ? "true" : "false");
        stopAllMotion();
        ROS_INFO("[DEBUG] stopAllMotion() 完成");
        ros::Duration(0.3).sleep();
        ROS_INFO("[DEBUG] executePureForwardMovement() 即将返回");
    }

    void stopAllMotion() {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        current_vel_ = stop_cmd;  // 更新 current_vel_ 为停止状态
        publishPlanningStatus();  // 发布停止状态
    }

    void executeLiftDown() {
        ROS_INFO("=== 开始下降 ===");
        
        const int MAX_WAIT = 120;
        int wait_count = 0;
        while (status_pub_.getNumSubscribers() == 0 && ros::ok() && wait_count < MAX_WAIT) {
            if (wait_count % 10 == 0) {
                ROS_INFO("等待 /planning_status 订阅者... (%d/%d)", wait_count, MAX_WAIT);
            }
            wait_count++;
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        if (status_pub_.getNumSubscribers() == 0) {
            ROS_WARN("[WARN] /planning_status 没有订阅者（等待%.1f秒超时），将继续执行", MAX_WAIT * 0.1);
        }
        
        lift_run_ = 0;
        rotate_run_ = 0;
        
        // 使用局部变量传递速度状态，避免修改全局变量
        geometry_msgs::Twist zero_vel;
        zero_vel.linear.x = 0.0;
        zero_vel.linear.y = 0.0;
        zero_vel.angular.z = 0.0;
        
        ros::Rate rate(10);
        int count = 0;
        const int max_count = 20;
        
        while (ros::ok() && count < max_count) {
            publishPlanningStatusWithVelocity(zero_vel);
            ros::spinOnce();
            rate.sleep();
            count++;
        }
        
        ROS_INFO("下降完成");
    }

    void executeRotate() {
        ROS_INFO("=== 开始旋转 ===");
        
        const int MAX_WAIT = 120;
        int wait_count = 0;
        while (status_pub_.getNumSubscribers() == 0 && ros::ok() && wait_count < MAX_WAIT) {
            if (wait_count % 10 == 0) {
                ROS_INFO("等待 /planning_status 订阅者... (%d/%d)", wait_count, MAX_WAIT);
            }
            wait_count++;
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        if (status_pub_.getNumSubscribers() == 0) {
            ROS_WARN("[WARN] /planning_status 没有订阅者（等待%.1f秒超时），将继续执行", MAX_WAIT * 0.1);
        }
        
        rotate_run_ = 1;
        lift_run_ = 0;
        
        // 使用局部变量传递速度状态，避免修改全局变量
        geometry_msgs::Twist zero_vel;
        zero_vel.linear.x = 0.0;
        zero_vel.linear.y = 0.0;
        zero_vel.angular.z = 0.0;
        
        ros::Rate rate(10);
        int count = 0;
        const int max_count = 20;
        
        while (ros::ok() && count < max_count) {
            publishPlanningStatusWithVelocity(zero_vel);
            ros::spinOnce();
            rate.sleep();
            count++;
        }
        
        ROS_INFO("旋转完成");
    }

    void loadGoals()
    {
        XmlRpc::XmlRpcValue goal_list;
        if (nh_.getParam("/get_move_base_data/goals", goal_list))
        {
            ROS_INFO("Loaded %d goals from param server.", goal_list.size());

            if (goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < goal_list.size(); ++i)
                {
                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.pose.position.x = static_cast<double>(goal_list[i]["x"]);
                    pose.pose.position.y = static_cast<double>(goal_list[i]["y"]);
                    pose.pose.orientation.w = 1.0;
                    goals_.push_back(pose);
                }
            }
        }
        else
        {
            ROS_WARN("No goals loaded from parameter server.");
        }
    }

    void sendNextGoal()
    {
        if (current_goal_index_ < goals_.size())
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = goals_[current_goal_index_];
            goal.target_pose.header.stamp = ros::Time::now();

            ac_.sendGoal(goal);
            goal_sent_ = true;
            move_base_state_ = "SENT";
            goal_reached_ = false;

            ROS_INFO("Sent Goal %ld: (%.2f, %.2f)", current_goal_index_,
                     goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        }
        else
        {
            ROS_INFO_THROTTLE(5.0, "All goals sent.");
            move_base_state_ = "DONE";
            goal_sent_ = true;
        }
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        current_vel_ = *msg;
        ROS_INFO_THROTTLE(1.0, "cmd_vel: linear=%.2f, lateral=%.2f, angular=%.2f",
                          msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ROS_INFO_ONCE("[DEBUG] 收到里程计数据: x=%.2f, y=%.2f", 
                      msg->pose.pose.position.x, msg->pose.pose.position.y);
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, odom_yaw_);

        odom_yaw_deg_ = odom_yaw_ * 180.0 / M_PI;
        if (odom_yaw_deg_ < 0)
            odom_yaw_deg_ += 360.0;
        
        odom_ready_ = true;
    }

    void publishPlanningStatus()
    {
        publishPlanningStatusWithVelocity(current_vel_);
    }

    void publishPlanningStatusWithVelocity(const geometry_msgs::Twist& vel)
    {
        ROS_INFO("[DEBUG] publishPlanningStatusWithVelocity() 被调用");
        
        decision_pkg::PlanningStatus msg;
        msg.linear_velocity = vel.linear.x;
        msg.lateral_velocity = vel.linear.y;
        msg.angular_velocity = vel.angular.z;
        msg.nav_status = nav_status_;
        msg.goal_reached = goal_reached_;
        msg.grip_run = grip_run_;
        msg.lift_run = lift_run_;
        msg.rotate_run = rotate_run_;
        msg.camera_y = camera_rel_body_pose_.position.y;
        msg.odom_yaw = odom_yaw_;

        ROS_INFO_THROTTLE(0.5, "[DEBUG] 发布 planning_status: linear=%.2f, lateral=%.2f, angular=%.2f, nav_status=%s, 订阅者=%d", 
                          msg.linear_velocity, msg.lateral_velocity, msg.angular_velocity, 
                          msg.nav_status.c_str(), status_pub_.getNumSubscribers());
        
        if (status_pub_.getNumSubscribers() == 0) {
            ROS_WARN_THROTTLE(1.0, "[WARN] /planning_status 没有订阅者！");
        }
        
        status_pub_.publish(msg);
    }

    void spin()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            updateCameraPose();

            if (!initial_action_completed_) {
                ROS_INFO_THROTTLE(1.0, "等待初始动作完成...");
            }
            else if (!goal_sent_ && current_goal_index_ < goals_.size())
            {
                sendNextGoal();
            }
            else if (goal_sent_)
            {
                actionlib::SimpleClientGoalState state = ac_.getState();
                if (state.isDone())
                {
                    move_base_state_ = state.toString();
                    goal_reached_ = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
                    current_goal_index_++;
                    goal_sent_ = false;
                    ROS_INFO("Goal %ld finished with state: %s (nav_status remains: %s)", 
                             current_goal_index_ - 1, state.toString().c_str(), nav_status_.c_str());
                }
            }
            else {
                // 所有目标已完成或没有目标，保持节点运行
                ROS_INFO_THROTTLE(5.0, "所有任务已完成，节点保持运行中...");
            }

            publishPlanningStatus();

            ros::spinOnce();
            rate.sleep();
        }
    }

    void updateCameraPose() {
        if (!tf_body_ready_) {
            ROS_DEBUG_THROTTLE(1.0, "TF body->camera_init 未就绪");
            return;
        }
        
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("body", "camera_init", ros::Time(0), ros::Duration(0.05));
            
            camera_rel_body_pose_.position.x = transform.transform.translation.x;
            camera_rel_body_pose_.position.y = transform.transform.translation.y;
            camera_rel_body_pose_.position.z = transform.transform.translation.z;
            camera_rel_body_pose_.orientation = transform.transform.rotation;
        } catch (tf2::TransformException &ex) {
            ROS_DEBUG_THROTTLE(1.0, "更新相机位姿失败: %s", ex.what());
        }
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("[DEBUG] === main函数开始 ===");
    
    ros::init(argc, argv, "goal_sender_action_client");
    ROS_INFO("[DEBUG] ROS初始化完成");
    
    try {
        DecisionNode node;
        ROS_INFO("[DEBUG] DecisionNode构造完成,进入spin()");
        node.spin();
    } catch (const std::exception& e) {
        ROS_ERROR("[ERROR] 节点异常退出: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("[ERROR] 节点未知异常退出");
        return 1;
    }
    
    ROS_INFO("[DEBUG] 节点正常退出");
    return 0;
}