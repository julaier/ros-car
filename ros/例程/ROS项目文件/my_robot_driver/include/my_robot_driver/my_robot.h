#ifndef MY_ROBOT_H
#define MY_ROBOT_H
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp> //多线程进行串口通信
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <atomic>
#include <mutex>
#include <cstring>
class MyRobot 
{
    public:   
    MyRobot();   // 构造函数
    ~MyRobot();  // 析构函数
    private:    
    void loadParams();   // 加载参数
    bool initSerial();   // 初始化串口 
    void serialReadThread(); // 串口异步读取线程    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);  //回调函数      // 数据处理与发布    
    void processFeedback(uint8_t* buf);   //处理发回的数据格式，解出线速度，角速度和偏航角
    void publishOdomAndTF(double vx, double vy,double wz, double yaw);  //将线速度角速度发送给里程计和TF
    ros::NodeHandle nh_, pnh_;        //nh 全局句柄 用来向cmd_vel发布
                                    // pnh 局部句柄，用来读取launch中的参数
    boost::asio::io_service io_service_;   //输入输出
    std::shared_ptr<boost::asio::serial_port> serial_ptr_;    //串口对象  
  // ROS 对象    
    ros::Subscriber cmd_sub_;  
    ros::Publisher odom_pub_;   
    tf2_ros::TransformBroadcaster tf_broadcaster_;  //发布世界坐标系和小车中心的位置关系
    // 位姿状态    
    double x_, y_, theta_;   //相对于世界坐标系的偏移
    ros::Time last_time_;      //时间戳，计算位移
     // 配置参数  
    std::string port_name_, odom_frame_, base_frame_;   //设置端口名，坐标系名
    int baud_rate_;//波特率

    std::atomic_bool running_{false}; // 线程运行标志
    boost::thread read_thread_; // 串口读取线程
    std::mutex serial_mtx_; // 串口写入互斥锁
};



#endif