# Turtlebot following a straight line by observing a marker
Group project of the subject 41014 Sensors and Control for Mechatronic Systems at UTS  


<!-- GROUP #6 MEMBERS -->
## Group #6 Members
|          Name       	  | 	ID    | Code contributed | Total |
| ---------------------- | --------- | ---------------- | ----- |
|     Sy Duy Nguyen 	  | 12699070  |     Control      |  60%  |
| Mohammad Sulaiman Khan | 13092489  | Marker detection |  40%  |
|  Huy Nhat Minh Nguyen  | 13734569  |       n/a        |  0%   |


<!-- GETTING STARTED -->
## Getting Started
To use the code, user must keep both main.m and ThresholdGenerator.m files in the same directory
(on the remote PC).  

The following toolboxes are required to execute this code on MATLAB:  
* []() ROS Toolbox (to send/receive ROS messages from the Turtlebot)
* []() Computer Vision Toolbox (to detect April tag markers)  

No other ROS packages required on the remote PC.  

When a ROS communication channel between the remote PC and the Turtlebot is established, 
main.m can then be executed that allows the Turtlebot to perform "Following a straight-line by
observing the marker" task.  



<!-- MAIN CODE STRUCTURE -->
## Main Code Structure
The main code is structured so that all variables that govern the outcome of the execution 
can be easily modified on the very top (before the while loop). These variables include ROS 
publisher and subscribers that require their inputs parameters to match with the ROS topics 
being advertised by the Turtlebot.  

Other variables are grouped under different region names such as Camera parameters, velocity gain 
and thresholds (i.e., transition, stop distances & boundary widths).  

**NOTE: Modifying variables under "Fixed" region is ill-advised as the entire code was developed heavily 
reliant on these values, therefore, it is suggested to leave these parameters untouched.    


Inside the while loop, the following tasks are being polled continuously:  
* []() Image data from the RealSense camera
* []() Marker (April tag) detection per each image received
* []() Front distance of the Turtlebot from the Laser scanner  


As the loop pumping data from the 3 tasks listed above, the P-controller (also implemented inside this loop) will 
combine those data to make the following decisions:  
* []() Switching stage (Coarse/Finer approach to the marker)
* []() Applying thresholds (Wide/Narrow boundary)  

Depending on which stage the robot is at, different size of boundaries (or corner container thresholds)

<!-- ADDITIONAL NOTES -->
## Additional notes
* []() To reduce program execution latency when receiving images, compressed format was chosen when 
subscribing to image topic.  
* []() Try/catch statement was implemented in the main loop to handle various errors (i.e., when the marker is 
not detected). Thus, prevent the Turtlebot from moving uncontrollably.
* []() Camera parameters are critical to ensure correct boundary for the marker is applied onto the image. User 
is expected to input correct width and height of the camera pixel size if they plan to not use the RealSense R200.

