# Robotics 2022/2023 first project
First project of the Robotics course at Polimi during academic year 2022-2023. The complete instructions are provided in the PDF in this repository.  
Summary of the project:  
Find the odometry of the EazyMile EZ10, vehicle with four wheels and autonomous guide. Front steer only.  
Data provided: wheel speed and steer angle, in bag file.
## what to do
Create a node "publisher" that publishes the odometry information with two kind of messages:
1. standard message `nav_msgs/Odometry` on the topic `/odometry`  
2. Custom messgae on the topic `custom_odometry` with the fields x, y, th, that are respectively, the x coordinate, y coordinate, angle. Include also the timestamp.  

The node should send the corresponding tf transform via `tf_broadcaster`. Then, expose a service (srv) called `reset_odom`, that resets the reference system (odometry), and returns true on success. Run the project with a launch file, called `first_project.launch`, where the parameters x,y,th are set to 0, and generate 4 static tf for the lasers.

