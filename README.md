exbot_sensors
=============

Contains a variety of sensors' drivers maintained by ExBot

Gendral
---------

To touch exbot_sensors, you need:

    cd ~/catkin_ws/src/
    git clone https://github.com/yuanboshe/exbot_sensors.git

Where catkin_ws is your catkin-type workspace of ROS. You should config your catkin workspace in your system first.

RPlidar
----------
###Bulid & Config

1. Download rplidar sdk from http://rplidar.robopeak.com/download.html , and unzip it into your *home/user* folder, e.g. *~/rplidar_sdk_v1.4.1* .

2. Open a terminal, and type the following commands to build rplidar lib:

        cd ~/rplidar_sdk_v1.4.1/sdk/sdk/
        make
    
    Then you can check if *~/rplidar_sdk_v1.4.1/sdk/output/Linux/Release/librplidar_sdk.a* exist.

3. Open *CMakelist.txt* from *exbot_sensors/rplidar* package:


        gedit ~/catkin_ws/src/exbot_sensors/rplidar/CMakeLists.txt


    And modify `set(RPLIDAR_DIR /home/exbot/rplidar_sdk_v1.4.1/sdk)` to your sdk folder path, and save.

4. Build *exbot_sensors/rplidar* package.

        cd ~/catkin_ws/
        catkin_make --pkg rplidar

###Test

1. Plug RPlidar USB in your PC.

2. To see RPlidar's com path, you may use `lsusb` and `ls /dev/ttyUSB*` .

3. Open prlidar launch file from *exbot_sensors/rplidar* package:
    
        gedit ~/catkin_ws/src/exbot_sensors/rplidar/launch/rplidar.launch
    
    And modify `<param name="com_path" type="string" value="/dev/ttyUSB0" />` to your com path, and save.

4. Open 3 terminals, and then type:

        roscore
        roslaunch rplidar rplidar.launch
        roslaunch rplidar view_rplidar.launch
    
Then you'll view the 2D points from rplidar in rviz.