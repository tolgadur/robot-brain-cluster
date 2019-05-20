# Robot Brain Cluster Readme

## Instructions
The Robot Brain Cluster code base is designed to facilitate our "Hide and Seek" demonstration which uses the specific DE NIRO hardware. The ROS remote launch and Brain Cluster files also had to be tailored to some degree to the specific desktops we used in the cluster. Therefore, our full code is not expected to run without access to original setup.

It should also be noted that the object (YOLO), face (dlib) and particular (find_object_2D) nodes require external libraries to be installed alongside.

However, the code base was designed to make each component as general as possible. This means the ROS packages for object, face and particular detection should all be applicable to any ROS system that publishes image data with only small modifications. Equally, many elements of the UI such as hot word detection, instruction encoding and display could all be reused for other projects on DE NIRO or a robot with similar hardware. 

Once set up correctly, the entire system runs with a simple call `roslaunch state_machine state_machine.launch`.
