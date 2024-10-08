# px4-offboard
aruco-marker detection & precision landing in gazebo
environment: unbuntu 20.04 & ros2 foxy

Author: Sunyong Hwang
Affiliation: Korea University, Mechanical Engineering
Reference: Jaeyoung-Lim/px4-offboard , yonseidrone.notion.site

## Setup
The steps below assume you have already installed:
ROS2 Foxy (Ubuntu 20.04), 
Gazebo, 
QGroundControl,
PX4-Autopilot.

FastDDS and FastRTPS should already be installed with Ubuntu 20.04, so start with installing Fast-RTPS-Gen.

### Installing Fast-RTPS-Gen
Refer to: https://docs.px4.io/main/en/dev_setup/fast-dds-installation.html
!Ubuntu 20.04: Fast DDS 2.0.2 (or later) and Fast-RTPS-Gen 1.0.4 (not later!).

Check if FastRTPS is installed:

```bash
dpkg -l | grep fastrt
```

You will see something like:

```bash
ii  ros-foxy-fastrtps                               2.1.2-1focal.20220829.174844               amd64        Implementation of RTPS standard.
ii  ros-foxy-fastrtps-cmake-module                  1.0.4-1focal.20220829.181444               amd64        Provide CMake module to find eProsima FastRTPS.
ii  ros-foxy-rmw-fastrtps-cpp                       1.3.1-1focal.20221012.224708               amd64        Implement the ROS middleware interface using eProsima FastRTPS static code generation in C++.
ii  ros-foxy-rmw-fastrtps-shared-cpp                1.3.1-1focal.20221012.221742               amd64        Code shared on static and dynamic type support of rmw_fastrtps_cpp.
ii  ros-foxy-rosidl-typesupport-fastrtps-c          1.0.4-1focal.20221012.215818               amd64        Generate the C interfaces for eProsima FastRTPS.
ii  ros-foxy-rosidl-typesupport-fastrtps-cpp        1.0.4-1focal.20221012.215633               amd64        Generate the C++ interfaces for eProsima FastRTPS.
```

Then you are all good!
​
Java is required to build and use eProsima's RTPS/DDS from IDL code generation tool - Fast-RTPS-Gen. Java JDK 11 is recommended, and it should have been installed as you installed PX4-Autopilot. (Right now I have JDK 13 and it works fine.)
Clone Fast-RTPS-Gen 1.0.4:

```bash
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
&& cd ~/Fast-RTPS-Gen/gradle/wrapper
​```

After that, modify the distribution version  of gradle inside the gradle-wrapper.properties file to gradle-6.8.3 such that the distributionUrl file becomes as follows:

```bash
distributionUrl=https\://services.gradle.org/distributions/gradle-6.8.3-bin.zip
```

You have to edit the file!
​
Now you should run the following commands:

```bash
cd ~/Fast-RTPS-Gen 
./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install
```

### Installing the px4-offboard
run the following commands:

```bash
cd ~
git clone https://github.com/janggimon/px4-offboard.git
cd ~/px4-offboard
colcon build
echo "source ~/px4-offboard/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installing the px4_ros_com and px4_msgs packages
The px4-offboard example requires the px4_ros_com bridge and px4_msgs definitions.
Run the following commands:

```bash
cd ~
mkdir px4_ros_com_ws
cd px4_ros_com_ws
mkdir src
cd src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
​```

Move to the px4_ros_com_ws workspace root directory and build them:

```bash
cd ~/px4_ros_com_ws
colcon build
​```

Add the setup.bash file to bashrc and source it:

```bash
echo "source ~/px4_ros_com_ws/install/local_setup.sh" >> ~/.bashrc
source ~/.bashrc
```

```bash
echo "source ~/px4_ros_com_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installing the micro_ros_agent  (one time setup)
Building the micro_ros_agent
Refer to:  Building micro-ROS-Agent (commands below is from Steve Henderson’s guide)

```bash
cd ~
mkdir ~/micro_ros_ws
cd micro_ros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
mkdir src
mv micro_ros_setup/ src/
colcon build
source install/local_setup.sh
sudo rosdep init
rosdep update
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
​```

Try running the agent:

```bash
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
​```

It should respond with:

```bash
[1675611895.560324] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1675611895.560477] info     | Root.cpp           | set_verbose_level        | logger setup     
```

Now source in bashrc:

```bash
echo "source ~/micro_ros_ws/install/local_setup.sh" >> ~/.bashrc
source ~/.bashrc
```
### Add downward camera into iris
modify ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf

find `<link name='base_link'> ... </link>` and then replace it with code below.

```python
<link name='base_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>1.5</mass>
        <inertia>
          <ixx>0.029125</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.029125</iyy>
          <iyz>0</iyz>
          <izz>0.055225</izz>
        </inertia>
      </inertial>
      <collision name='base_link_inertia_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.47 0.47 0.11</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <visual name='base_link_inertia_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://iris/meshes/iris.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <gravity>1</gravity>
      <velocity_decay/>
      <sensor name="downward_camera" type="camera">
        <pose>0 0 -0.15 0 1.5708 0</pose>
        <camera>
          <horizontal_fov>1.3089</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100.0</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>30.0</updateRate>
          <cameraName>drone_camera</cameraName>
          <topicName>/camera/image_raw</topicName>
        </plugin>
      </sensor>
    </link>
```

### Add Aruco marker model
Unzip 'marker1.zip' and move it to '/home/sunyong/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/'.

Then yon can add aruco marker 'marker1' in Gazebo - insert tab. It's 5X5 marker with white contour.

![Marker1](https://github.com/user-attachments/assets/f83b0f15-e14a-43a5-8080-dba2adc1be0f)


If you are running this on a companion computer to PX4, you will need to build the package on the companion computer directly. 

## Running

### Software in the Loop
You will make use of 4 different terminals to run the offboard demo.

On the first terminal,

```bash
cd ~/micro_ros_ws
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```

On the second terminal,

```bash
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
make px4_sitl gazebo
```

On the third terminal,

```bash
cd ~/px4-offboard
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 launch px4_offboard offboard_position_control.launch.py
```

On the fourth terminal,

```bash
./QGroundControl.AppImage
```

Then insert an aruco marker on the ground in Gazebo and let the iris(drone) take off in QGroundControl. Test the detection and precision landing.

If you want to check the ros graph, run the following command in home directory:
```bash
ros2 run rqt_graph rqt_graph
```
Then you can see the graph as below:
![rosgraph2](https://github.com/user-attachments/assets/64dec5bc-b562-4518-bcec-08634ad7bf97)

## How to modify controller node
If you want to modify control system,
Modify `class ArucoMarkerDetector(Node)` and `class DroneControllerNode(Node)`
in `~/px4-offboard/px4_offboard/offboard_control.py`

--------------------------------------------------not revised below-----------------------------------------------------------
### Hardware

This section is intended for running the offboard control node on a companion computer, such as a Raspberry Pi or Nvidia Jetson/Xavier. You will either need an SSH connection to run this node, or have a shell script to run the nodes on start up. 

If you are running this through a UART connection into the USB port, start the micro-ros agent with the following command

```
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600 -v
```
If you are using a UART connection which goes into the pinouts on the board, start the micro-ros agent with the following comand
```
micro-ros-agent serial --dev /dev/ttyTHS1 -b 921600 -V
```

To run the offboard position control example, run the node on the companion computer
```
ros2 launch px4_offboard offboard_hardware_position_control.launch.py
```

### Visualization parameters

The following section describes ROS2 parameters that alter behavior of visualization tool.


#### Automatic path clearing between consequent simulation runs

Running visualization node separately from the simulation can be desired to iteratively test how variety of PX4 (or custom offboard programs) parameters adjustments influence the system behavior. 

In such case RVIZ2 is left opened and in separate shell simulation is repeadetly restarted. This process causes paths from consequent runs to overlap with incorrect path continuity where end of previous path is connected to the beginning of new path from new run. To prevent that and clear old path automatically on start of new simulation, set `path_clearing_timeout` to positive float value which corresponds to timeout seconds after which, upon starting new simulation, old path is removed.

As an example, setting the parameter to `1.0` means that one second of delay between position updates through ROS2 topic will schedule path clearing right when next position update comes in (effectively upon next simulation run).


To enable automatic path clearing without closing visualization node set the param to positive floating point value:
```
ros2 param set /px4_offboard/visualizer path_clearing_timeout 1.0
```

To disable this feature set timeout to any negative floating point value (the feature is disabled by default):
```
ros2 param set /px4_offboard/visualizer path_clearing_timeout -1.0
```
