# multi_diffcar_ws  
基于ROS实现多差速无人车编队控制  

# 1 依赖  
除了需要安装ros环境外，还需要安装eigen、casadi数学运算库  

# 2 拷贝源码与编译  
git clone https://github.com/quyinsong/multi_diffcar_ws.git  
cd multi_diffcar_ws  
catkin_make  
source devel/setup.bash  

# 3 运行代码  
运行多机器人仿真环境（gazebo与rviz）：roslaunch mydiffcar_gazebo multi_diffcar_gazebo.launch  
运行编队控制器：roslaunch nmpc_ctr test_formation.launch  

# 4 rviz为领航车发布2D nav goal  

![image](https://github.com/user-attachments/assets/2c6c5c29-bf1a-41a6-a9c9-c9d4be99b9a3)



