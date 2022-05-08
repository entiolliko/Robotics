# Robotics Project 1
**Team members** <br>
Alen Kaja (10696919) <br>
Entiol Liko ( 10xxxxxx)

# Structure of the package

We provided a package containing different **folders**:
- **cfg**: containing the python parameters.cfg file for the dynamic reconfigure of the integration type; <br> 
 this file contains an enum with two values, the default one is Euler integration, you can switch between Euler and RK
- **launch**: containing the launch file of the project that starts all the necessary nodes, rviz and rqt_reconfigure window: <br>
-> the *initial pose* of the robot is set by three parameters X, Y, Theta <br>
-> the link between *world* and *odom* frame is set by a static transformation <br>

- **msg**: containing the custom messages:<br>
-> *custom.msg* is the structure of the message that publishes the x,y and theta of the computed odometry and the Ground Truth pose, in order to compare these values in the Calibration node <br> 
-> *speeds.msg* is the structure of the message that publishes the linear velocity of the wheels in rpm; <br>
it contains also a header for synchronization and 4 fields which represent the four wheels: rpm_fl, rpm_fr, rpm_rr, rpm_rr <br>

- **srv**: containing the reset pose service: <br>
-> *reset.srv* that changes the pose of the robot to any given pose (X, Y, Theta) <br>

- **src**: containing the following files (nodes): <br>
   
  -> **data**: header file which contains the five given parameters of the robot: the radius, the wheel positions along x and y, the gear ratio and the encoders resolution.

  -> **TwistCalculator**: this node is used to listen to the messages coming from the encoders of the robot; it subscribes to the */wheel_states* topic of the bag and it computes the robot linear speeds *Vx*, *Vy* and the angular speed *⍵*; this is performed using the mecanum wheel kinematic model and the wheel velocities are obtained from the encoder ticks provided by the topic; finally it publishes a message of type  "TwistStamped" on the */cmd_vel* topic.

	-> **OdometryCalculator**: this node is used to compute the odometry of the robot, to publish all the related messages and to expose the service mentioned before.<br>
The node reads the messages published on topic */cmd_vel* node and based on the value of the param "odomMethod" it computes the odometry of the robot using Euler or Runge-Kutta integration.  After this it publishes the odometry on the */odom* topic and broadcasts the tf transformation.<br>
For the integration type, the node uses the dynamic reconfigure method to set the integration type; the default value is "Euler" and whenever the user changes the param a callback is executed, changing the value of the field "integrationType" of the class OdometryCalculator.
The node contains also the callback for the service "reset", that sets the pose of the robot equal to the value x, y and theta that the user passes when calls the service.
		
  -> **WheelSpeedCalculator**: this node computes wheel speeds from *Vx*, *Vy*, *⍵* previously calculated in the TwistCalculator node, which is done by reversing the formula used before. It subscribes to the */cmd_vel* topic and publishes custom messages of type "speeds" on topic */wheels_rpm*.

   -> **InitialPoseClient**: this node is a client to run the "reset" position service: it handles the request of reset and displays messages of success or failure of the service.
   
	-> **Calibration**: this is an utility node that publishes messages of type "custom" in order to plot the Ground Truth pose of the robot both with the computed odometry, accordingly on the */customOdom* and the */customGT* topics.

## Parameters description
- X, Y, Theta are three parameters set to zero in the launch file and for different initialization the user can change them in the launch file or call the apposite service.
- odomMethod is the parameter that can be dynamically reconfigured to change integration method between Euler and RK by setting it to 0 or 1.

## TF Tree
The tf tree is composed of three transformation frames world -> odom ->base_link: a static transformation links the world frame to the odom frame, the relation is thought in order to match the gt_pose data with odom calculation. <br>
The transformation between odom and base_link is the one responsible for the real localization of the robot in the space.

## Custom messages structure
-> *custom.msg*, that is used in Calibration node to  publish the *x*, *y* and *θ* of the computed odometry and the Ground Truth pose. <br>
-> *speeds.msg*  message that publishes the linear velocity of the wheels in rpm. It contains also a header for synchronization and four float64 values which represent the four wheels: rpm_fl ( front left ), rpm_fr ( front right ), rpm_rr (rear right), rpm_rr (rear left).


## How to start/use the code

The launch file opens all the necessary tools:

The command "roslaunch project1 launcher.launch" 
-> starts all the necessary nodes;
-> opens rviz, that shows the gt_pose ( green arrow), our odometry(red arrow) published from the "/odom" topic.
-> opens rqt_reconfigure_params GUI window, from where the integration type can be changed.

At this point the nodes are ready to receive messages from a rosbag and calculate odometry based on the data.
A bag can be played by running "rosbag play [my_bag.bag]"
It will be possible to check on rviz the robot motion or by opening plotjuggler and plot the x, y, theta of the robot


The service can be used with these commands:

->"rosservice call /reset [*your_x*] [*your_y*]  [*your_theta*]" 
to set the initial pose of the robot equal to the ground truth pose.<br>
Alternatively you can change these parameters in the launch file.

->"rosrun InitialPoseClient [*your_x*] [*your_y*]  [*your_theta*]", equivalent to the previous command and it uses the InitialPoseClient.

