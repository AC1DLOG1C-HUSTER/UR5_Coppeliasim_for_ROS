# 注意事项：

本项目参考开源项目https://github.com/YoussefChouj/UR5-Path-Planning-and-Grasping-using-Coppeliasim-

以完成ros仿真与编程课堂作业

在其基础上加入ros话题通信机制

在此感谢原作者的无私开源。

文件夹一级目录中的Cam.py文件等，并非py文件，而是.ttt文件中child script的内容备份，便于处理，以LUA格式编写。


# 环境要求：

ubuntu20.04

coppeliaSim

ros1

python运行环境及一些前置库文件，包括


# 运行步骤：

执行步骤：

***1：启动vrep仿真软件***

终端1执行

./coppeliaSim.sh 

在项目文件夹中找到ur5_coppelia_ws\src\UR5-Path-Planning-and-Grasping-using-Coppeliasim-的.ttt文件并导入scene

***2：启动ros核心***

终端2执行

roscore

***3：ros工作空间编译***

终端3执行

cd ~/ur5_coppelia_ws
catkin_make
source devel/setup.bash

***3.5：如果有需求***

可先在scene中启动，再在终端3中直接用python执行项目文件夹中的

ur5_coppelia_ws\src\UR5-Path-Planning-and-Grasping-using-Coppeliasim-\Path_planning_for_a_UR5_using_RG2_gripper_kinpy_OMPL_2.py

回答两个问题后（推荐先yes 后 no），便会进行运动学计算，生成运动轨迹。

当然，整个过程耗时长，并且完整的结果文件已经生成好（两个json文件）

因此，不执行也是可以的

***4：正式运行***

终端4执行

roslaunch ur5_ros_bridge ur5_ros_bridge.launch

此时切换到coppeliasim界面，便能看到机械臂运行。
