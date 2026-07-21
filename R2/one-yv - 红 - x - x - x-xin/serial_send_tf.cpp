#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <serial/serial.h>
#include <string>
#include <cstring>
#include <cmath>

class SerialSender
{
public:
    // 构造函数：通过节点句柄获取参数，初始化串口
    SerialSender(ros::NodeHandle &nh)
    {
        // 从参数服务器获取配置（优先使用launch文件中设置的参数）
        std::string port;
        int baudrate;

        // 获取串口端口参数，默认值为/dev/ttyUSB0
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        // 获取波特率参数，默认值为115200
        nh.param("baudrate", baudrate, 115200);

        try
        {
            // 配置串口参数
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 超时1000ms
            ser_.setTimeout(timeout);

            // 打开串口
            ser_.open();

            if (ser_.isOpen())
            {
                ROS_INFO("Serial port %s opened at %d baud (configured via launch file).",
                         port.c_str(), baudrate);
            }
            else
            {
                ROS_ERROR("Failed to open serial port: %s", port.c_str());
                ros::shutdown();
            }
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("Serial port error: %s", e.what());
            ros::shutdown();
        }
    }

    ~SerialSender()
    {
        if (ser_.isOpen())
        {
            ser_.close(); // 关闭串口
        }
    }

    // 地图坐标系位姿回调
    void mapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        pose_map_ = *msg;
        received_map_ = true;
        trySend();
    }

    // 相机坐标系位姿回调
    void camCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        pose_cam_ = *msg;
        received_cam_ = true;
        trySend();
    }

private:
    serial::Serial ser_; // ROS serial包的串口对象
    geometry_msgs::PoseStamped pose_map_;
    geometry_msgs::PoseStamped pose_cam_;
    bool received_map_ = false;
    bool received_cam_ = false;

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
    void trySend()
    {
        if (!received_map_ || !received_cam_)
            return;

        // 数据缓冲区，包含预留扩展空间
        uint8_t buffer[128];
        size_t len = 0;

        // 起始符
        buffer[len++] = 0xAA;

        // 提取位姿数据
        float map_x = pose_map_.pose.position.x;
        float map_y = pose_map_.pose.position.y;
        float map_yaw = getYawDeg(pose_map_.pose.orientation);
        float cam_x = pose_cam_.pose.position.x;
        float cam_y = pose_cam_.pose.position.y;
        float cam_yaw = getYawDeg(pose_cam_.pose.orientation);

        // 填充数据到缓冲区（按照原始格式）
        // 1. 线速度和角速度设为0
        float zero_float = 0.0f;
        std::memcpy(buffer + len, &zero_float, sizeof(float));
        len += sizeof(float); // linear_vel
        std::memcpy(buffer + len, &zero_float, sizeof(float));
        len += sizeof(float); // angular_vel

        // 2. 导航状态和目标到达标志设为0
        buffer[len++] = 0x00; // nav_status_code
        buffer[len++] = 0x00; // goal_reached

        // 3. 将6个float数据填入相机坐标、里程计偏航角和预留浮点位置
        std::memcpy(buffer + len, &map_x, sizeof(float));
        len += sizeof(float); // camera_x
        std::memcpy(buffer + len, &map_y, sizeof(float));
        len += sizeof(float); // camera_y
        std::memcpy(buffer + len, &map_yaw, sizeof(float));
        len += sizeof(float); // odom_yaw
        std::memcpy(buffer + len, &cam_x, sizeof(float));
        len += sizeof(float); // reserved_float1
        std::memcpy(buffer + len, &cam_y, sizeof(float));
        len += sizeof(float); // reserved_float2
        std::memcpy(buffer + len, &cam_yaw, sizeof(float));
        len += sizeof(float); // reserved_float3

        // 4. 预留uint8_t设为0
        buffer[len++] = 0x00; // reserved_uint8

        // 计算数据段的偶校验（从起始符后到预留空间结束）
        uint8_t parity = calculateEvenParity(buffer + 1, len - 1); // 排除起始符
        buffer[len++] = parity;

        // 结束符
        buffer[len++] = 0xEE;

        // 发送数据
        if (ser_.isOpen())
        {
            ser_.write(buffer, len);
            ROS_INFO_THROTTLE(1.0, "Sent TF packet to serial (port: %s)", ser_.getPort().c_str());

            // 可选：打印发送的数据详情
            ROS_DEBUG_THROTTLE(1.0, "Sent data: map_x=%.2f, map_y=%.2f, map_yaw=%.2f, cam_x=%.2f, cam_y=%.2f, cam_yaw=%.2f",
                               map_x, map_y, map_yaw, cam_x, cam_y, cam_yaw);
        }

        // 重置标志位
        received_map_ = false;
        received_cam_ = false;
    }
    // 四元数转航向角（度）
    float getYawDeg(const geometry_msgs::Quaternion &q)
    {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return static_cast<float>(std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_send_tf");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // 私有节点句柄，用于读取当前节点的参数

    // 初始化SerialSender，通过私有节点句柄读取参数
    SerialSender sender(pnh);

    // 订阅位姿话题
    ros::Subscriber sub_map = nh.subscribe("/body_in_map", 1, &SerialSender::mapCallback, &sender);
    ros::Subscriber sub_cam = nh.subscribe("/body_in_camera_init", 1, &SerialSender::camCallback, &sender);

    ros::spin();
    return 0;
}
