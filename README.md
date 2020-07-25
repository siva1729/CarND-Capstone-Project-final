# Self-Driving Car System Integration: Team ExiaCar

### Team Members


| First Name        | Last Name           | Email                     |
| ----------------- |:-------------------:| -------------------------:|
| Shreyansh         | Bhatt               | shreyansh@knoesis.org     | 
| Youssef           | Ben Dhieb           | y.bendhieb@instadeep.com  |
| Sreedhar          | Sivalingala         | sivalingala@yahoo.com     |
| Eddie             | Forson              | eddie.forson@gmail.com    |

[//]: # (Image References)
[saimg]: ./readme_imgs/final-project-ros-graph-v2.png "System_Architecture"
[tdnimg]: ./readme_imgs/tl-detector-ros-graph.png "Traffic Detection Node"
[ssdimg]: ./readme_imgs/ssd_architecture.png "SSD Mobilenet"
[simtl_img1]: ./readme_imgs/simtl_img1.png "Simulator TL Image1"
[simtl_img2]: ./readme_imgs/simtl_img2.png "Simulator TL Image2"
[rwtl_img1]: ./readme_imgs/rwtl_img1.png "Real World TL Image1"
[rwtl_img2]: ./readme_imgs/rwtl_img2.png "Real World TL Image2"
[rwtl_img3]: ./readme_imgs/rwtl_img3.png "Real World TL Image3"
[wpimg]: ./readme_imgs/waypoint-updater-ros-graph.png "Waypoint Update Node"
[csimg]: ./readme_imgs/dbw-node-ros-graph.png "DBW Node"

## Overview
System Integration project is the final project of the Udacity Self-Driving Car Engineer Nanodegree: Programming a Real Self-Driving Car (Carla). Our Team implemented the core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. This software system will be deployed on Carla (Udacity’s Self Driving Lincoln MKZ) to autonomously drive it around a test track.

## Project Setup:
The project was developed using Ubuntu Linux (the operating system of Carla) and Robot Operating System(ROS) Framework. The system integration project used its own simulator which will interface with ROS code and has traffic light detection. The Term3 simulator can be found [here](https://github.com/udacity/CarND-Capstone/releases). 
  * Ubuntu 16.04 with ROS Kinetic
  * TensorFlow 1.3

## System Architecture Diagram:
For this project, used ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following! And tested the code using a simulator on tracks 1 and 2.

The following is a system architecture diagram showing the ROS nodes and topics used in the project. The ROS nodes and topics shown in the diagram are described briefly in the Code Structure section below.
![alt text][saimg]


# Software/Code Structure:
Here is a brief overview of the repo structure, along with descriptions of the ROS nodes. The code  for the project is contained within the /ros/src/ directory. Within this directory, are the following ROS packages and implements the nodes in Perception, Planning and Control subsytems:

## Perception Subsytem/ Traffic Light Detection Node
 Perception subsytem is meant to work as sensor subsytem handling for the car to sense the surrounding world for upcoming traffic lights and obstacles and publishes relevant information to other subsystems. The main implementation of this subsytem is the Traffic Light Detection node that is supposed to detect and classify the upcoming taffic light and take necessary actions of slowing down and stopping if the light is RED and so on. For this project the obstacle detection is not implemented. Here is the interface diagram of Traffic Dection Node.
 ![alt text][tdnimg]
   
 This is implemented in the package /ros/src/tl_detector/ and this package contains the Traffic Light (TL) detection node: tl_detector.py. This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.
 The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following. You will build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within ../tl_detector/light_classification_model/tl_classfier.py.
   
 ### Traffic Light Detection and Classification:
 The main part of the TL Node is a Traffic Light Detection and Classifier that has been implemented using
 [Tensor Flow Object Dection API](https://github.com/tensorflow/models/tree/master/research/object_detection).
 Traffic light detection node takes a captured image as input and produces the bounding boxes as the output to be fed into the classification model. After much research, we decided to use TensorFlow Object Detection API, which is an open source framework built on top of TensorFlow to construct, train and deploy object detection models. The Object Detection API also comes with a collection of detection models pre-trained on  the COCO dataset that are well suited for fast prototyping. For us the speed and being light weight was important as Vehicle HW is close to light weifht Mobile HW. Decided to go with a lightweight model: ssd_mobilenet_v1_coco (speed 30 ms, COCO mAP 21) that is based on Single Shot Multibox Detection (SSD) framework with minimal modification. 
  The speed & mAP for different models can be found 
 [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
 ![alt text][ssdimg]
 
  The COCO dataset contains images of 90 classes ranging from vehicle to human. The index for traffic light is 10. Though this is a general-purpose detection model (not optimized specifically for traffic light detection), we find this model sufficiently met our needs, achieving the balance between good bounding box accuracy (as shown in the following figure) and fast running time. 
  For our purpose, we have additionally trained this model separately for simulator and real world using the simulator images, and real world images. found on the Udacity slack captured by a former SDCND student who labeled images from the simulator as well the real-life track.
  [here](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view)
  
  The SSD Mobilenet has been retrained for Traffic Light images and was done separately for Simulator images and real world images as the image input between simulator camera images and on board Carla camera images would be quite different. Training for simulator classifier has been done using simulator images and we beleive it might be overfitting to a certain extent and this is ok as the images used for inference would not be different from training images. The network has been trained for 30K epochs and the frozen model can be found under
  deep_learing/models/frozen_graphs/sim_mobilenets_ssd_30k_epochs_frozen_inference_graph.pb  Here is output of the TL Detecton/Classifier for the Simulator camera images 
  ![alt text][simtl_img1]
  ![alt text][simtl_img2]
  
  Similarly for real worl camera image handling the network has been trained with dataset found on the Udacity slack captured by a former SDCND student who labeled images from the simulator as well the real-life track.
  [here](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view)
  And the trained inference model can be found at deep_learing/models/frozen_graphs/real_mobilenets_ssd_38k_epochs_frozen_inference_graph.pb 
  And here are output images for inference runon camera images
  ![alt text][rwtl_img1]
  ![alt text][rwtl_img2]
  ![alt text][rwtl_img3]

## Planning Subsystem/Waypoint Nodes
The Planning subsytem determines the path planning for the vehicle based on its current position, velocity and the feedback from TL about any upcoming lights. It publishes a list of waypoints to the control subsystem that uses this information to generate vehicle commands.

This package ros/src/waypoint_updater/ contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.
![alt text][wpimg]

Waypoint Loader node loads a CSV file that contains all the waypoints along the track and publishes them to the topic /base_waypoints (implemented by Udacity). CSV files for waypoints are differnt for simulator and realworld and loaded based on lauch config. And thw Waypointer update node handles bulk of the path planning and it subscribes to topics to get the entire list of waypoints, the vehicle’s current position, and the state of upcoming traffic lights. This waypoints list is stored only once as they do not change. 

Waypoints are calculated continuously to make sure the car follows the track and after we find the closest waypoint to the vehicle’s current position we determine a list next 200 waypoints to follow. As we keep getting TL info for upcoming traffic lights, if we detect then we calculate the velocity for each waypoint to decelarate such that the car comes to a full stop at the stop line. 

The final list of waypoints is published on the /final_waypoints topic.

## Control Subsystem/Controller nodes
The Control subsytem uses the Drive-By-Wire (DBW) node and publishes the vehicle's throttle, steering and brakes commands based on the waypoint list provided by planning subsytem.
![alt text][csimg]

Waypoint Follower Node parses the list of waypoints to follow and publishes proposed linear and angular velocities to the /twist_cmd topic (provided by Udacity).Drive By Wire (DBW) Node is the interface between SW and HW of the Carla self driving vehicle’s system.  
To control the throttle, steering and brakes, use different controller modules (reference implementation provided by Udacity). Throttle controller is a simple PID controller that compares the current velocity with the target velocity and adjusts the throttle accordingly. The throttle gains were tuned using trial and error for allowing reasonable acceleration without oscillation around the set-point. The values for the PID controller are tuned for proper throttle handling to avoid lags.
The Steering Controller outputs steering angle based on the linear and angular velocities using the vehicle’s steering ratio and wheel-base length. Limit the maximum linear and angular acceleration rates so that vehicle runs smoothly. The output of the steering controller is passed through a Low Pass filter to filter the jitter/noise from the command messages. And the Braking Controller outputs proportionally brake based on the difference in the vehicle’s current velocity and the proposed velocity. 

We found that the default controllers provided by Uacity worked well except we had to tune the PID controller.

## Conclusion
Overall it was challenging/good project to introduce us to the ROS and the underlying SW Achitecture overview.
We were able to run the car successfully on both tracks (on a PC with GPU) and were able to detect the Traffic lights on the images from the ROS bag of Carla camera. There were situations where TL detection/classifier needed to improve detection of Green lights. But we are confident the code will work ok on the Carla on-site testing.


# Instructions for setup and how to run the repo
### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
