#include <ros/ros.h>
#include "nmpc_ctr.h"
#include <geometry_msgs/PoseStamped.h> //获取rviz目标点数据
#include <nav_msgs/Odometry.h> //获取里程计数据
#include <geometry_msgs/Twist.h> //发布cmd_vel
#include <nav_msgs/Path.h> //发布预测轨迹点
#include <visualization_msgs/Marker.h> //以箭头形式在rviz中可视化目标点
#include <tf/tf.h>


/**
 * 该控制器控制跟随车robot3跟随robot1
 */

//通过订阅无人小车当前里程计话题获取小车实时状态
class CurrentStatesListener
{
public:
    Eigen::Matrix<float,3,1> m_current_states;

public:
    CurrentStatesListener();
    ~CurrentStatesListener();
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
};

CurrentStatesListener::CurrentStatesListener()
{

}

CurrentStatesListener::~CurrentStatesListener()
{

}

void CurrentStatesListener::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x, y;
    double roll, pitch, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    //四元数转欧拉角
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); //取出方向存储于四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    m_current_states<<x,y,yaw;
}

//主函数
int main(int argc, char **argv)
{
    //初始化ros节点
    ros::init(argc, argv, "formation_ctr_3");

    ros::NodeHandle n;

    //创建NMPC控制器
    int predict_step = 50;
    float sample_time = 0.1;
    NMPC nmpc_ctr(predict_step, sample_time);

    //设定期望最终位置与控制量
    Eigen::Vector3f goal_states;
    goal_states<< 0,0,0;
    nmpc_ctr.set_goal_states(goal_states);

    //创建监听对象，获取领航车状态
    CurrentStatesListener mlistener1;

    ros::Subscriber sub1 = n.subscribe("/robot1/odom",100,&CurrentStatesListener::callback,&mlistener1);

    //创建监听对象，获取robot2跟随车状态
    CurrentStatesListener mlistener2;

    ros::Subscriber sub2 = n.subscribe("/robot3/odom",100,&CurrentStatesListener::callback,&mlistener2);

    //创建控制量发布对象
    ros::Publisher controls_pub =n.advertise<geometry_msgs::Twist>("/robot3/cmd_vel",100);

    //创建预测轨迹发布对象
    ros::Publisher path_pub =n.advertise<nav_msgs::Path>("/robot3/predict_trajectory",100);

    //创建rviz目标点可视化发布对象
    ros::Publisher marker_pub =n.advertise<visualization_msgs::Marker>("/robot3/goal_marker",100);

    // 设置循环的频率
    ros::Rate loop_rate(10);

    //设置在robot1坐标系中的期望编队队形
    float xd_from_rb1 = -0.001;
    float yd_from_rb1 = 0.001;

    while(ros::ok())
    {
        //循环回调函数
        ros::spinOnce();

        //获取当前ros时间
        ros::Time current_time = ros::Time::now();

        //获取领航者小车robot1状态
        Eigen::Matrix<float,3,1> robot1_states = mlistener1.m_current_states;
        float rb1_x =  robot1_states[0];
        float rb1_y =  robot1_states[1];
        float rb1_psi =  robot1_states[2];

        //获取跟随者小车robot3状态
        Eigen::Matrix<float,3,1> robot3_states = mlistener2.m_current_states;
        float rb3_x =  robot3_states[0];
        float rb3_y =  robot3_states[1];
        float rb3_psi =  robot3_states[2];

        //计算小车robot3期望到达的位置点--将robot1中的期望坐标变换到map坐标系
        float xd_from_map, yd_from_map, psid_from_map;
        // xd_from_map = xd_from_rb1*std::cos(rb1_psi)-yd_from_rb1*std::sin(rb1_psi)+rb1_x;
        // yd_from_map = xd_from_rb1*std::sin(rb1_psi)+yd_from_rb1*std::cos(rb1_psi)+rb1_y;
        xd_from_map = rb1_x-1;
        yd_from_map = rb1_y-1;
        psid_from_map = rb1_psi;
        goal_states<<xd_from_map,yd_from_map,psid_from_map;

        //将期望点发布到NMPC控制器
        nmpc_ctr.set_goal_states(goal_states);

        //发布目标点数据，在rviz中可视化
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        marker.color.r = 1.0f; // 点为红色
        marker.color.a = 1.0;
        geometry_msgs::Point p1, p2;
        p1.x = goal_states[0];
        p1.y = goal_states[1];
        p1.z = 0;
        p2.x = goal_states[0]+0.2*std::cos(goal_states[2]);
        p2.y = goal_states[1]+0.2*std::sin(goal_states[2]);
        p2.z = 0;
        marker.points.push_back(p1) ;
        marker.points.push_back(p2) ;
        marker_pub.publish(marker) ;

        //调用nmpc求解
        nmpc_ctr.opti_solution(mlistener2.m_current_states);
        //获取控制量
        Eigen::Vector2f nmpc_controls = nmpc_ctr.get_controls();
        //获取预测轨迹点
        std::vector<Eigen::Matrix<float,3,1>> predict_trajectory;
        predict_trajectory = nmpc_ctr.get_predict_trajectory();
        int len_predict_trajectory = predict_trajectory.size();
        
        //发布预测轨迹点
        nav_msgs::Path path;
        path.header.stamp=current_time;
        path.header.frame_id="map";
        geometry_msgs::PoseStamped this_pose_stamped;

        for(int k=0;k<len_predict_trajectory;k++)
        {
            this_pose_stamped.pose.position.x = predict_trajectory.at(k)[0];
            this_pose_stamped.pose.position.y = predict_trajectory.at(k)[1];
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(predict_trajectory.at(k)[2]);
            this_pose_stamped.pose.orientation.x = goal_quat.x;
            this_pose_stamped.pose.orientation.y = goal_quat.y;
            this_pose_stamped.pose.orientation.z = goal_quat.z;
            this_pose_stamped.pose.orientation.w = goal_quat.w;
            this_pose_stamped.header.stamp=current_time;
            this_pose_stamped.header.frame_id="odom";
            path.poses.push_back(this_pose_stamped);
        }
        path_pub.publish(path);

        //发布求解得到的控制量
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = nmpc_controls[0];
        twist_msg.linear.y = 0;
        twist_msg.linear.z = 0;
        twist_msg.angular.x = 0;
        twist_msg.angular.y = 0;
        twist_msg.angular.z = nmpc_controls[1];

        controls_pub.publish(twist_msg);

        //休眠
        loop_rate.sleep();
    }
 

  return 0;
}