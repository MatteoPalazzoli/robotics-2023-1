# Report
## Kinematics used
We've chosen a Tricycle model with the bicycle approximation, because the speed was a single variable and not double (left-right). We assumed that the speed is referring to the front wheel.
## Covariance matrices
We noticed that in the `nav_msgs/Odometry` there are two fields requesting 6x6 covariance matrices. Since the documention on the ROS website is not very explicative, we assumed that these refer to some error-related information about the pose and twist.
Since we don't know neither an error measure nor a probability distribution, we chose to set them to all zeroes.
