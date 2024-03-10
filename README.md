# t265_to_mavros

## T265定点的坐标系及T265在无人机上的安装说明
板卡端坐标系是东北天，飞控端坐标系是北东地。  
T265在无人机上的安装，只需要保证偏航和横滚与机身一致，俯仰可以在+90~-90度之间，当然建议保持正朝前，因为T265有imu，会使得输出位姿与重力方向对齐，所以T265俯仰+90~-90度之间转动，T265输出的位姿话题依旧是东北天坐标系，可以直接使用。  
所以T265的位姿数据传入飞控后，QGC地面站端看到的无人机偏航是朝东的。  
此时无人机往机头方向移动，板卡端会看到位姿的x会正方向增大，而在QGC地面站端看飞控内的位置数据是y正方向变大。  

## 功能包部署
```
mkdir t265_to_mavros_ws/src
cd  t265_to_mavros_ws/src
git clone https://github.com/maxibooksiyi/t265_to_mavros.git
cd ..
catkin_make
```
## T265定点
### 命令启动
可以选择通过sh脚本一次性启动所有命令，或者自己依次手动输入命令启动。
#### sh脚本一次性启动
进入到 ~/t265_to_mavros_ws/src/t265_to_mavros/sh 文件夹内，在此文件夹内打开终端，把  t265_localization.sh  脚本拖到此终端内，并敲回车，即可开始执行  t265_localization.sh  脚本，将T265定点的所有节点全部起起来。
#### 依次输入命令启动
打开一个终端运行下面命令
```
sudo chmod 777 /dev/ttyTHS0
roslaunch mavros px4.launch
```
打开一个终端运行下面命令
```
roslaunch realsense2_camera rs_t265.launch
```
打开一个终端依次运行下面命令
```
source ~/t265_to_mavros_ws/devel/setup.bash
rosrun offboard_pkg t265_to_mavros
```
### 命令启动完成后的操作
运行完之后，可以遥控器切定点，解锁，然后手动起飞，即可飞成T265定点。
## T265指点
### 命令启动
可以选择通过sh脚本一次性启动所有命令，或者自己依次手动输入命令启动。
#### sh脚本一次性启动
进入到 ~/t265_to_mavros_ws/src/t265_to_mavros/sh 文件夹内，在此文件夹内打开终端，把  t265_setpoint.sh  脚本拖到此终端内，并敲回车，即可开始执行  t265_setpoint.sh  脚本，将T265指点走正方形的所有节点全部起起来。
#### 依次输入命令启动
就是在T265定点命令运行基础上再打开一个终端多运行下面两条命令（在无人机已经飞完T265定点降落回起飞点，运行T265定点命令的终端依旧在运行的前提下）
```
source ~/t265_to_mavros_ws/devel/setup.bash
roslaunch offboard_pkg setpoint.launch
```
### 命令启动完成后的操作
在setpoint.launch为默认的没有更改的情况下，上面T265指点命令启动后，先遥控器依次切定点，解锁，切offboard，然后无人机会先自动起飞到0.4米高度，再依次往前飞1米，往左飞1米，往后飞1米，往右飞1米回到起飞点上方，然后自动降落并上锁。

### 说明
飞T265指点之前注意确认场地大小，把无人机摆放在合适的位置，确认可以让无人机安全飞完正方形。

可以在setpoint.launch里更改飞行的高度，正方形边长，以及是否自动解锁切offboard。

setpoint.launch内容如下
参数 height 是设置无人机飞正方形的高度，单位米
参数 side 是设置无人机飞正方形的边长，单位米
参数 auto_arm_offboard 是设置无人机是否自动解锁切offboard，如果值为true，则是setpoint.launch起起来后无人机会自动解锁切offboard然后自动起飞开始飞正方形，如果值为false，则是setpoint.launch起起来后需要先通过遥控器手动解锁切offboard，然后无人机才开始起飞飞正方形。
```
<launch>
	
   <node pkg="offboard_pkg" type="offboard_node" name="offboard_node" output="screen">

   <param name="height"     value="0.4"/>
   <param name="side"    value="1"/>
   <param name="auto_arm_offboard"   type="bool"  value="false"/>

   </node>
</launch>

```
