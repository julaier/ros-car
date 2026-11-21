#include "my_robot_driver/my_robot.h"
#include <tf2/LinearMath/Quaternion.h> // tf2 相关头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2 与 geometry_msgs 之间的转换
#include <cmath>
#include <atomic> // C++11 及以上的原子类型 用于线程安全的标志
#include <mutex> // C++11 及以上的互斥锁 用于保护串口写入

// ---------------- 协议常量 ----------------
static constexpr uint8_t HEADER = 0xFF; // 协议头
static constexpr uint8_t TAIL   = 0xFE; // 协议尾

// 发送长度: 1(头) + 4(vx) + 4(vy) + 4(wz) + 1(尾) = 14
static constexpr uint8_t TX_PACKET_LEN = 14; 

// 接收长度: 1(头) + 4(vx) + 4(vy) + 4(wz) + 4(yaw) + 1(尾) = 18
// (假设你的 STM32 发回来的 yaw 也是 4字节 float)
static constexpr uint8_t RX_PACKET_LEN = 18;
// ---------------- 构造/析构 ----------------

MyRobot::MyRobot()
    : pnh_("~"), // pnh_局部句柄加载初始位置，位于世界中心.且设置线程安全的运行标志为false
      x_(0.0), y_(0.0), theta_(0.0),
      running_(false) {

    loadParams();

    if (initSerial()) {
        running_ = true; // 启动线程前设置标志

        // 保存线程句柄，析构时 join
        read_thread_ = boost::thread(boost::bind(&MyRobot::serialReadThread, this));

        // 以10hz的频率获取cmd_vel消息，回调函数是cmdVelCallback
        cmd_sub_  = nh_.subscribe("/cmd_vel", 10, &MyRobot::cmdVelCallback, this);
        // 以50hz频率发布odom消息
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
        // 当前时间
        last_time_ = ros::Time::now();
        ROS_INFO("Robot Driver Core Module Loaded.");
    } else {
        ROS_ERROR("Robot Driver initSerial() failed.");
    }
}

MyRobot::~MyRobot() {
    // 让线程退出，线程状态改为false
    running_ = false;

    // 关闭串口，让 read_some 早点抛异常/返回
    try {
        if (serial_ptr_ && serial_ptr_->is_open()) {
            serial_ptr_->close();
        }
    } catch (...) {}

    // 等待读线程结束
    try {
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    } catch (...) {}
}



void MyRobot::loadParams() {
    // 提供默认值
    pnh_.param<std::string>("robot_port",  port_name_, "/dev/robot_base");
    pnh_.param<int>("baud_rate",           baud_rate_, 115200);
    pnh_.param<std::string>("odom_frame",  odom_frame_, "odom");
    pnh_.param<std::string>("base_frame",  base_frame_, "base_footprint");
}

// ---------------- 串口初始化 ----------------
bool MyRobot::initSerial() {
    try {
        // 串口配置
        serial_ptr_ = std::make_shared<boost::asio::serial_port>(io_service_);
        serial_ptr_->open(port_name_);
        serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        return true;
    } catch (std::exception& e) {
        ROS_ERROR("Serial Error: %s", e.what());
        return false;
    }
}

// ---------------- 串口读线程（最小增强版） ----------------
// 状态机解析协议
void MyRobot::serialReadThread() {
    uint8_t temp_buf[1024];
    uint8_t rx_buf[64];

    uint8_t state = 0;
    uint8_t rx_num = 0;
    uint8_t data_len = 0;
  
    // ros::ok() 和 running_ 保证线程安全
    while (ros::ok() && running_) {
        try {
            // 防空指针/未初始化， 防串口已关闭 ，防cpu空转
            if (!serial_ptr_ || !serial_ptr_->is_open()) {
                ros::Duration(0.05).sleep();
                continue;
            }
            // 读串口
            size_t bytes_read = serial_ptr_->read_some(
                boost::asio::buffer(temp_buf, sizeof(temp_buf)));
            // 状态机解析
            for (size_t i = 0; i < bytes_read; ++i) {
                uint8_t res = temp_buf[i];

                if (state == 0) {
                    if (res == HEADER) {
                        rx_buf[0] = res;
                        rx_num = 1;
                        state = 1; // 找到包头，开始接收数据
                    }
                } 
                else if (state == 1) {
                    rx_buf[rx_num++] = res;
                    
                    // 当收到足够长度的数据时 (RX_PACKET_LEN = 18)
                    if (rx_num == RX_PACKET_LEN) {
                        if (rx_buf[RX_PACKET_LEN - 1] == TAIL) {
                            // 尾部校验成功，处理数据
                            processFeedback(rx_buf);
                        } else {
                            // 尾部不对，说明发生错位，丢弃该包
                            ROS_WARN_THROTTLE(1.0, "Serial RX frame error: Tail mismatch");
                        }
                        // 无论成功还是失败，都重置状态机准备接下一个包
                        state = 0; 
                        rx_num = 0;
                    }
                }
            }
        } catch (std::exception& e) {
            if (running_) {
                ROS_ERROR("Serial Read Thread Error: %s", e.what());
                ros::Duration(0.1).sleep();
            }
        }
    }
}
// ---------------- 写串口（加个锁更稳） ----------------

void MyRobot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!serial_ptr_ || !serial_ptr_->is_open()) return;

    // 打包协议 
    uint8_t tx_buf[TX_PACKET_LEN];
    float vx = static_cast<float>(msg->linear.x );
    float vy = static_cast<float>(msg->linear.y );
    float wz = static_cast<float>(msg->angular.z );

    tx_buf[0] =HEADER;

    memcpy(&tx_buf[1], &vx, sizeof(float));
    memcpy(&tx_buf[5], &vy, sizeof(float));
    memcpy(&tx_buf[9], &wz, sizeof(float));

    tx_buf[13] = TAIL;
    try {
        // 加锁保护串口
        std::lock_guard<std::mutex> lk(serial_mtx_);
        boost::asio::write(*serial_ptr_, boost::asio::buffer(tx_buf, TX_PACKET_LEN));
    } catch (boost::system::system_error& e) {
        ROS_ERROR("Failed to write to serial port: %s", e.what());
    }
}

// ---------------- 解包反馈（加长度保护） ----------------

void MyRobot::processFeedback(uint8_t* buf) {
    if(buf[0] != HEADER || buf[RX_PACKET_LEN - 1] != TAIL) {
        return;
    }

    float vx, vy, wz, yaw;

    memcpy(&vx,&buf[1], sizeof(float));
    memcpy(&vy,&buf[5], sizeof(float));
    memcpy(&wz,&buf[9], sizeof(float));
    memcpy(&yaw,&buf[13], sizeof(float));
   

    publishOdomAndTF(static_cast<double>(vx), static_cast<double>(vy),static_cast<double>(wz), static_cast<double>(yaw));
}

// ---------------- 里程计（dt保护 + 姿态一致用yaw） ----------------

void MyRobot::publishOdomAndTF(double vx, double vy, double wz, double yaw) {
    ros::Time now = ros::Time::now();

    if (last_time_.isZero()) {
        last_time_ = now;
        return;
    }

    double dt = (now - last_time_).toSec();

    // ✅ dt 异常保护（避免突然跳变导致位置炸掉）
    if (dt <= 0.0 || dt > 0.2) {
        last_time_ = now;
        return;
    }

    // ✅ 姿态一致：直接用 IMU yaw 来旋转速度（毕设更稳）
    double heading = yaw;

    double delta_x = (vx * std::cos(heading) - vy * std::sin(heading)) * dt;
    double delta_y = (vx * std::sin(heading) + vy * std::cos(heading)) * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ = yaw;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_.sendTransform(tf_msg);

    nav_msgs::Odometry odom;
    odom.header = tf_msg.header;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf_msg.transform.rotation;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

    // 协方差（毕设可简化）
    odom.pose.covariance[0]  = 1e-3;
    odom.pose.covariance[7]  = 1e-2;
    odom.pose.covariance[35] = 1e-3;

    odom_pub_.publish(odom);

    last_time_ = now;
}