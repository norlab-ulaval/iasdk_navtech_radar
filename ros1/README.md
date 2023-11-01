# Navtech Example ROS Implementation

## Requirements

* Visual Studio 2019
* ROS 1 SDK
* OpenCV 4.2.0 (for visualisation)

## Summary
This SDK is an example of how radar data can be extracted from the radar and some example applications of how you may want to process the data. 

## Build Instructions

Using a ROS command prompt goto the ros folder in this repository

```bash
catkin_make install
```
## Running The Application

Ensure Roscore is running before executing this command

```bash
call devel\setup.bat
```

## Mobile-Platform
This application is a tool for processing the raw radar data (ideal for moving platforms). 

It works by using dynamic reconfiguration to allow the user flexibility in analysing the data the radar is sending. The topics published by the application are a pointcloud of the filtered data, with the option of identifying targets with bounding boxes, an image topic with identified targets highlighted and another image topic showing the intermediate filtering stage to show the user what’s happening in the background. 

It also gives the user the option to publish the filtered data as a laser scan.
BE AWARE: This loses the depth information available from the sensor. The laser scan returns the range of the first object the radar sees.This can then be used as the laser scan in combination with a tf topic containing odometry information of your robot to create a map and localize with gmapper.
In rviz, view the laser scan topic by typing in the header frame "navtech_laserscan".

The Mobile-Platform application subscribes to the ‘Navtech/Polar’ topic published by Talker1 which contains the raw polar data from the radar. Therefore, this app can be run in real time or to post-process the data for analysis and configuring the parameters to best suit your needs by replaying a rosbag file containing the ‘Navtech/Polar’ topic.

To start the real-time processing of data from the radar, make sure roscore is running, open a ros terminal following the instructions above and type the following command to start the data publishing data and launch the app. Within the code, there is the option to perform azimuth adaptive thresholding. This reduces false alarms from radar signal artefacts and saturations.

```bash
call devel\setup.bat (OR source devel/setup.bash)
roslaunch talker1.launch
```
In a new ROS terminal
```bash
call devel\setup.bat (OR source devel/setup.bash)
rosrun nav_ross Mobile_Platform
```


Or, for post processing data stored in a rosbag file, you can start the Mobile-Platform node alone. Do this by opening a ros terminal as usual, call devel\setup.bat and then roslaunch Mobile_Platform.launch.

```bash
catkin_make
roslaunch Mobile_Platform.launch
```

Navigate to plugins->Reconfigure->Dynamic Reconfigure
Select Mobile_Platform from the list

From here you can load a rosbag file for playback (if not using realtime data) as well as dynamically reconfigure the parameters in the parameter server.

### Parameters:

Min/max angle – adjust to throw away data past these angles

LaserScan2_Map - Select this to publish data as a LaserScan. Be aware of loss of data. 

radarimage - check this box to publish radar data as an image topic

Threshold value – Threshold value for the processed data (60 is low, 150 is high for object detection)

Pointcloud – select if you’d like the pointcloud topic published	

pcl_Threshold value – Threshold value for the visualization of pointcloud data (60 is low, 150 is high)

boundingboxes2 – Select this if you’d like targets to have a bounding box drawn around them, published as a marker array

MinAreaRect - Select this if you'd like bounding boxes to be minimum area fitted quadrilaterals.

Dilation value – Number of pixels apart that objects have to be to be classed as a single object

Gaussian value – Size of gaussian filter applied to the raw data for smoothing

Adaptive threshold - Select this if you'd like to use local thresholding. This will calculate the local mean pixel intensity value and keep the central point only if is above the mean + a constant. BE AWARE - this algorithm has no guard cells, and therefore will be less efective at identifying large objects (without setting adaptive_size to a large value).

Level_above_average - This is the amount a point has to be above the local mean to be considered above the threshold

adaptive_size - This is the width and height in pixels of the area in which to calculate the local mean. 

Grid – turns grid on or off

Grid_stepSize – step size of grid (metres)

Grid_opacity – opacity of grid

Colormap – we suggest 2, but this changes the colour scheme of your processed image

Finally, to visualize the data, open rviz and subscribe to the topics you would like to see. The header frame for the data is 'navtech'.


### Movement Detector
Movement Detector, (if fft is switched on in the launch file) is an example of a subscriber to the raw radar data. It filters the data and will detect any non-stationary key features in the output using an azimuth-based CFAR detection algorithm. The application will then visualize the data and highlight the features in green boxes. This will only work when the radar itself is stationary as it builds up a clutter map of the world.

Ensure Roscore is running before executing this command

```bash
roslaunch Movement_Detector.launch
```

## Subscribing to Topics With RQT Visualizer

Both the image and pointcloud topics can be subscribed to using the rqt visualizer. For the pointcloud, open a new ROS command prompt.
```bash
rviz rviz
```
*	Fixed Frame name set to ‘navtech’
*	Zoom out to a distance ~ 260 
*	Add, Pointcloud2 or Laserscan or Marker as desired, OK
*	Topic, select from list
*	Time Stamp Information:
*	Time stamp header is the time at the end of each full 360 scan. The seconds and split seconds (standard ROS header format) are using the UNIX epoch of the 1st Jan 1970 00:00:00, where split seconds is the number of nanoseconds

The image topic can simply be selected in the list of topics after adding an image.

## Launch Files