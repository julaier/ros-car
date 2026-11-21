#include "my_robot_driver/my_robot.h"
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"my_robot_driver_node");

    MyRobot robot_driver;//创建对象

    ros::spin();//循环等待回调函数
    return 0;
}