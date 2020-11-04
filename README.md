ackermann_vehicle
=================

This project includes many aspect and all of them are done by ROS/Gazebo environment and the programming language used is Python.
Starting by launching 3 ackermann vehicles in one Gazebo environmeent and providing a path planning of lane changing of the leader car, the leader will move according to this path using Fuzzy Logic Control, and the 2 followers will follow each other using Goal to Goal controller.

A full simulation of this process can be done by setting up your workspace and build this package in it. After that, you can "roslaunch ackermann_vehicle_gazebo main1.launch" , this will launch your gazebo enivronment with the three cars already available.

After that you can call each car's controllers on individual basis, so you may open a new termianl and type For the leader movement:- "rosrun ackermann_vehicle_gazebo new_leader.py" For the first follower movement:- "rosrun ackermann_vehicle_gazebo Follower1.py" For the second follower movement:- "rosrun ackermann_vehicle_gazebo Follower2.py"


I think you will be watching how the platooning system is very smooth in following each other. However, it may diverge from each other afteer the path ends (after the lance change has done), I do not care about that as it is used as a part of a bigger project.
I hope everything is crystal clear and if you need any further illustration, do not hesitate to contact me.


Gasser Elazab
gasser.elazab@gmail.com

