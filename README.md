# 221e MULTI-SENSOR DRIVER FOR ROS

The official ROS driver for the 221e MUlti-SEnsor (MUSE) device.

Package available for:
- ROS Noetic - Ubuntu 20.04;
- ROS Noetic - Windows 10.

## 1 - Installation

***Step 1 : Install the ROS distribution***

a) Ubuntu: 
- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on [Ubuntu 20.04](https://www.ubuntu-it.org/download);

b) Windows: 
- Install [ROS Noetic](https://wiki.ros.org/Installation/Windows) on Windows 10;

If you are exploiting Windows, we assume you are using [Microsoft Visual Studio 2019](https://docs.microsoft.com/en-us/visualstudio/releases/2019/release-notes). Remember to
- include the "Desktop development with C++" workload;
- in the Individual Components, select "Windows 10 SDK". 

Moreover, remember to start your installation routine from the ROS command short cut.

***Step 2: Install lib221e***

lib221e is the C++ library for the 221e Communication Protocol. Follow the installation instructions available [here](https://github.com/221eROS/lib221e.git).

You can use git from Command Line. Otherwise, several free and commercial GUI tools are available. E.g., for the Windows platform, you can download a GUI from [here](https://www.git-scm.com/downloads/guis).

In the same way, in Windows, you can install your library via [Visual Studio](https://visualstudio.microsoft.com/it/downloads/) [installed in step 1(b)]. A possible installation procedure follows:
- open Visual Studio with administrator privileges;
- choose to open up your library from your local folder;
- open the ```Developer Command Prompt``` from ```Visual Studio-->Tools-->Command Line```;
- launch ```cmake ..```;
- launch ```cmake --build . --target INSTALL --config Release```.


***Step 3 : Create a [Catkin](http://wiki.ros.org/catkin#Installing_catkin) Workspace***

a) Ubuntu:

Open a terminal. From the ```home``` directory:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_init_workspace
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
Other details are available [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

b) Windows:

Open your ROS short cut terminal. In your ```c:\``` directory:

```sh
$ mkdir c:\catkin_ws\src
$ cd c:\catkin_ws
$ catkin_make
$ devel\setup.bat
```
Other details are available [here](https://github.com/Brabalawuka/RosOnWindows).

You can decide to locate your workspace in other directories.

If you need the Ninja build tool, go to the [releases page](https://github.com/ninja-build/ninja/releases) and download a suitable binary for Windows. Place ```ninja.exe``` in a suitable spot, e.g., ```C:\Ninja```. Make sure that CMake can find ```ninja.exe``` by adding ```C:\Ninja``` to your ```%PATH%```.

***Step 4: Clone muse_v2_driver in your Catkin Workspace

a) Ubuntu:
```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/221eROS/muse_v2_driver.git
$ catkin_make
```

b) Windows:
```sh
$ cd c:\catkin_ws\src
$ git clone https://github.com/221eROS/muse_v2_driver.git
$ cd ..
$ catkin_make
```
## 2 - ROS for Windows: additional tips for Visual Studio 2019 [No mandatory!]

- We suppose you have already installed Visual Studio 2019in step 1(b);
- From the ROS shortcut of Step 1(b), enter ```devenv``` to start your Visual Studio IDE; 
- Choose to open up your catkin workspace from your local folder;
- As Visual Studio is using different settings for CMake projects, we need to configure the CMake file. Thus, click ```Project --> CMake Settings for Project``` and edit the JSON version of your Cmake settings file as follows:

```sh
{
  "configurations": [
    {
      "name": "x64-Debug",
      "generator": "Ninja",
      "configurationType": "RelWithDebInfo",
      "inheritEnvironments": [ "msvc_x64_x64" ],
      "buildRoot": "C:\\catkin_ws\\build",
      "installRoot": "C:\\catkin_ws\\install",
      "cmakeCommandArgs": "DCATKIN_DEVEL_PREFIX=C:\\catkin_ws\\devel",
      "ctestCommandArgs": ""
    }
  ]
}
```
In this way, Cmaking and building your project from Visual Studio will be equal to a catkin_make outside the Visual Studio environment. 
You can also open the Command Line directly from the Visual Studio ```Tools``` button.

## 3 - Usage 

### a) Device connection
Switch on your MUSE V2 device. From your catkin workspace, launch the following command to connect to your sensor and enable its ROS node:
```sh
$ roslaunch muse_v2_driver muse_v2.launch
```
Among others, this launch file allows you to set:
- port name (Default: COM5);
- baudrate (Default: 115200).

### b) Device shutdown
Assuming the ROS node of your MUSE V2 sensor is active [step 3(a)], launch the following command to switch off your device:
```sh
$ roslaunch muse_v2_driver shutdown.launch
```

### c) Configuration
***Get configuration parameters.*** Assuming the ROS node of your MUSE V2 sensor is active [step 3(a)], launch the following command from your catkin workspace:
```sh
$ roslaunch muse_v2_driver get_configuration_params.launch
```
 It will output the configuration parameters of your device:
- gyroscope full-scale;
- accelerometer full-scale;
- accelerometer HDR full-scale;
- magnetometer full-scale;
- log mode;
- log frequency.

By default, ```launch/get_configuration_params.launch``` enables the request of all parameters. You can freely *enable/disable* each parameter transmission by setting to *true/false* its corresponding argument in this launch file.

***Set configuration parameters.*** To change the configuration parameters of your MUSE V2 sensor, you can edit the  ```muse_v2_config.yaml```  file in the ```config``` folder. The following parameters can be configured:
- gyroscope full-scale value (Default: 0 Available: {500, 1000, 2000, 4000});
- accelerometer full-scale value (Default: 0 Available: {2, 4, 6, 8, 16});
- accelerometer HDR full-scale value (Default: 0 Available: {100, 200, 400});
- magnetometer full-scale value (Default: 0 Available: {2, 4, 8, 12});
- log mode (Default: UINT8_MAX Available: {0, 1, 2, 3, 4, 5});
- log frequency (Default: UINT(_MAX Available: <= 250).
	
Assuming the ROS node of your MUSE V2 sensor is active [step 3(a)], launch the following command to write the new parameters on your device:
```sh
$ roslaunch muse_v2_driver set_configuration_params.launch
```

### d) Calibration
***Get calibration params.*** Assuming the ROS node of your MUSE V2 sensor is active [step 3(a)], launch the following command from your catkin workspace:
```sh
$ roslaunch muse_v2_driver get_calibration_params.launch
```
It will output the calibration parameters of your device:
- gyroscope offset;
- accelerometer calibration parameters;
- magnetometer calibration parameters.

Such parameters will be stored in ```config/muse_v2_calib.txt```. By default, the launch file enables the request of all parameters. You can freely *enable/disable* each parameter retrieval by setting to *true/false* its argument in ```launch/get_calibration_params.launch```.

### e) Memory

Assuming the ROS node of your MUSE V2 sensor is active [step 3(a)], the following command lets you retrieve log data from your device memory:

```sh
$ roslaunch muse_v2_driver logger.launch
```
According to the flagged parameter, the device will output:
- the available memory [kB];
- a boolan value attesting that the memory has been correctly erased;
- the list of log files;
- all logs of a selected log file;
- all logs of all stored log files.

Focusing on the log files, 
- *get_files:=true* will output the list *[(1, file_name_1.txt), ...,  (n, file_name_n.txt)]* of log files stored in your device;
- *read_file:=i* (with i ={1, ..., n}) will store the content of file i-th;
- *read_memory:=true* will store the content of all log files.

Log files are stored in the ```config``` folder. 


### f) Data acquisition
***Published topics.*** Raw data are published at the following topics:
- *imu/data* (sensor_msgs/Imu): quaternion, angular velocity, and linear acceleration;
- *imu/acceleration* (geometry_msgs/Vector3Stamped): calibrated linear acceleration;
- *imu/angular_velocity* (geometry_msgs/Vector3Stamped): calibrated angular velocity;
- *imu/mag* (geometry_msgs/Vector3Stamped): calibrated magnetic field;
- *imu/quaternion* (geometry_msgs/QuaternionStamped): sensor orientation expressed in quaternion;
- *imu/rpy* (geometry_msgs/Vector3Stamped): sensor orientation expressed in Roll-Pitch-Yaw.

***Start acquisition.*** We assume that the ROS node of your MUSE V2 device is active [step 3(a)]. To start raw data transmission, launch the following command from your catkin workspace:
```sh
$ roslaunch muse_v2_driver start_transmission.launch
```
 By default, ```start_transmission.launch``` starts to publish the following data:

- IMU;
- quaternion;
- RPY;
- angular velocity;
- acceleration;
- magnetic field.

You can freely *enable/disable* each data publisher by setting to *true/false* its corresponding argument. The launch file also enables to choose the desired sampling *frequency* (Default: 150 Hz - Admitted range: [1-150] (Hz)).

***Plots.***  While streaming raw data, you can plot them by means of
```sh
$ roslaunch muse_v2_driver plot.launch
```
To enable the activation of one or more plots, you should set to *true* its corresponding argument.

***Raw data subscriber.*** While streaming raw data, you can subscribe to the *publisher* node and publish data back on your terminal. Assuming your transmission node is active, launch the following command from your catkin workspace:
```sh
$ roslaunch muse_v2_driver raw_data_subscriber.launch
```
***Stop acquisition.*** If your acquisition node is active and you want to stop data transmission:
```sh
$ roslaunch muse_v2_driver stop_transmission.launch
```

### g) Miscellaneous

***Get battery charge and voltage.***  Assuming the ROS node of your MUSE V2 sensor is active [step 3(a)], type the following command from your catkin workspace:
```sh
$ roslaunch muse_v2_driver battery.launch
```
The node will send back the battery charge, its voltage, or both depending on whether the arguments *charge* and *voltage* are set to *true*.

## 4 - Example

The following example shows how to ask a 221e MUSE V2 sensor to stream raw data and enable the 3D visualization of the IMU plot.
- Open one terminal and, from your catkin workspace, type:
```sh
$ roslaunch muse_v2_driver muse_v2.launch
```
- Open a second terminal and, from you catkin workspace, type:
```sh
$ roslaunch muse_v2_driver start_transmission.launch
```
- Open a third terminal and, from you catkin workspace, type:
```sh
$ roslaunch muse_v2_driver plot.launch imu:=true
```
- On your desktop, the 3D view of your sensor will appear:

<p align="center">
  <img src="/imgs/imu_rviz_output_example.png" />
</p>

It displays the orientation of the IMU using a box as well as and coordinate axes. The acceleration can also be visualized using a vector.
