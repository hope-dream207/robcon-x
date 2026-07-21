#include <ros/ros.h>
#include <decision_pkg/PlanningStatus.h>
#include <serial/serial.h>
#include <string>
#include <cstring>
#include <cmath>

class SerialMoveBaseSender
{
public:
    // 构造函数：初始化串口
    SerialMoveBaseSender(ros::NodeHandle& nh)
    {
        // 从参数服务器获取串口配置
        std::string port;
        int baudrate;
        
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        nh.param("baudrate", baudrate, 115200);

        try {
            // 配置串口参数
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            
            // 打开串口
            ser_.open();
            
            if (ser_.isOpen()) {
                ROS_INFO("Serial port %s opened at %d baud for move_base data", 
                         port.c_str(), baudrate);
            } else {
                ROS_ERROR("Failed to open serial port: %s", port.c_str());
                ros::shutdown();
            }
        } catch (serial::IOException& e) {
            ROS_ERROR("Serial port error: %s", e.what());
            ros::shutdown();
        }
    }

    ~SerialMoveBaseSender()
    {
        if (ser_.isOpen()) {
            ser_.close();
        }
    }

    // PlanningStatus消息回调函数
    void planningStatusCallback(const decision_pkg::PlanningStatus::ConstPtr& msg)
    {
        ROS_INFO_ONCE("[DEBUG] 串口节点首次收到 /planning_status 消息");
        ROS_INFO_THROTTLE(0.5, "[DEBUG] 收到 planning_status: linear=%.2f, lateral=%.2f, angular=%.2f", 
                          msg->linear_velocity, msg->lateral_velocity, msg->angular_velocity);
        current_status_ = *msg;
        trySendData();
    }

private:
    serial::Serial ser_;
    decision_pkg::PlanningStatus current_status_;

    // 计算偶校验位
    uint8_t calculateEvenParity(const uint8_t* data, size_t length)
    {
        uint8_t parity = 0;
        for (size_t i = 0; i < length; ++i) {
            // 计算每个字节的奇偶性
            for (int j = 0; j < 8; ++j) {
                parity ^= (data[i] >> j) & 0x01;
            }
        }
        return parity; // 偶校验位（0表示偶数个1，1表示奇数个1）
    }

    // 尝试发送数据
    void trySendData()
    {
        if (!ser_.isOpen()) return;

        // 数据缓冲区，包含预留扩展空间
        uint8_t buffer[128];
        size_t len = 0;

        // 起始符
        buffer[len++] = 0xAA;

        // 提取需要发送的数据
        float linear_vel = current_status_.linear_velocity;
        float lateral_vel = current_status_.lateral_velocity;
        float angular_vel = current_status_.angular_velocity;
        uint8_t nav_status_code = getStatusCode(current_status_.nav_status);
        uint8_t goal_reached = current_status_.goal_reached ? 1 : 0;
        uint8_t grip_run = current_status_.grip_run;     // 爪子: 0关闭/1打开
        uint8_t lift_run = current_status_.lift_run;       // 升降: 0下降/1上升
        uint8_t rotate_run = current_status_.rotate_run; // 旋转: 0停止/1旋转
        float camera_y = current_status_.camera_y;
        float odom_yaw = current_status_.odom_yaw;

        // 填充当前使用的数据到缓冲区
        std::memcpy(buffer + len, &linear_vel, sizeof(float)); len += sizeof(float);
        std::memcpy(buffer + len, &lateral_vel, sizeof(float)); len += sizeof(float);
        std::memcpy(buffer + len, &angular_vel, sizeof(float)); len += sizeof(float);
        buffer[len++] = nav_status_code;
        buffer[len++] = goal_reached;
        // 机械臂控制字段（替换原camera_x位置，共3字节，节省1字节转到预留位）
        buffer[len++] = grip_run;
        buffer[len++] = lift_run;
        buffer[len++] = rotate_run; // 旋转: 0停止/1旋转
        std::memcpy(buffer + len, &camera_y, sizeof(float)); len += sizeof(float);
        std::memcpy(buffer + len, &odom_yaw, sizeof(float)); len += sizeof(float);

        // 预留扩展空间：3个float和2个uint8_t（原camera_x节省的1字节转到此处）
        float reserved_float1 = 0.0f;  // 预留扩展用
        float reserved_float2 = 0.0f;  // 预留扩展用
        float reserved_float3 = 0.0f;  // 预留扩展用
        uint8_t reserved_uint8_1 = 0x00; // 预留扩展用（新增，来自原camera_x节省的1字节）
        uint8_t reserved_uint8_2 = 0x00; // 预留扩展用
        
        std::memcpy(buffer + len, &reserved_float1, sizeof(float)); len += sizeof(float);
        std::memcpy(buffer + len, &reserved_float2, sizeof(float)); len += sizeof(float);
        std::memcpy(buffer + len, &reserved_float3, sizeof(float)); len += sizeof(float);
        buffer[len++] = reserved_uint8_1;
        buffer[len++] = reserved_uint8_2;

        // 计算数据段的偶校验（从起始符后到预留空间结束）
        uint8_t parity = calculateEvenParity(buffer + 1, len - 1); // 排除起始符
        buffer[len++] = parity;

        // 结束符
        buffer[len++] = 0xEE;

        // 发送数据
        ser_.write(buffer, len);
        
        // 打印发送信息（不包含预留空间以保持日志简洁）
        ROS_INFO_THROTTLE(1.0, "Sent move_base data: linear=%.2f, lateral=%.2f, angular=%.2f, status=%s, reached=%d, grip=%d, lift=%d, rotate=%d, camera_y=%.2f, odom_yaw=%.2f, parity=0x%02X",
                         linear_vel, lateral_vel, angular_vel, 
                         current_status_.nav_status.c_str(),
                         goal_reached, grip_run, lift_run, rotate_run, camera_y, odom_yaw, parity);
    }

    // 将导航状态字符串转换为状态码
    uint8_t getStatusCode(const std::string& status)
    {
        // 三种母状态
        if (status == "ONE") return 1;
        if (status == "TWO") return 2;
        if (status == "THREE") return 3;
        
        // 以下状态已注释，保留为参考
        /*
        if (status == "IDLE") return 0;
        if (status == "SENT") return 1;
        if (status == "PENDING") return 2;
        if (status == "ACTIVE") return 3;
        if (status == "SUCCEEDED") return 4;
        if (status == "PREEMPTED") return 5;
        if (status == "ABORTED") return 6;
        if (status == "REJECTED") return 7;
        if (status == "DONE") return 8;
        */
        
        return 1;
    }
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "serial_send_move_base");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 初始化发送器
    SerialMoveBaseSender sender(pnh);

    // 订阅move_base状态话题
    ros::Subscriber sub = nh.subscribe("/planning_status", 10, 
                                      &SerialMoveBaseSender::planningStatusCallback, 
                                      &sender);

    ros::spin();
    return 0;
}