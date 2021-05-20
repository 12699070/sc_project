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

When a ROS communication channel between the remote PC and the Turtlebot is established, 
main.m can then be executed to allow the Turtlebot to perform "Following a straight-line by
observing the marker" task.


<!-- MAIN CODE STRUCTURE -->
## Main Code Structure
The main code is structured so that all variables that govern the outcome of the execution 
can be easily modified on the very top (before the while loop). These variables include ROS 
publisher and subscribers that are required their inputs parameters to match with ROS topics 
being advertised by the Turtlebot.  

Other variables are grouped under different region names such as Camera parameters, velocity gain 
and thresholds (i.e., transition, stop distances & boundary widths).  

**NOTE: Modifying variables under "Fixed" region is ill-advised as the entire code was developed heavily 
reliant on these values, therefore, it is suggested to leave these parameters untouched.  




<!-- CODE DESCRIPTION -->
## Code description



