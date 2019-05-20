# Robot Brain Cluster Readme

## Abstract 
Physical robots, whether they be for industrial, personal or scientic use are often limited by computational constraints imposed by a range of factors. Our goal in this project was to relax these constraints on Imperial College London's Design Engineering Natural Interaction Robot (DE NIRO) through implementing and building on top of a cluster of computers that provides the robot with opportunities to delegate processing to one of multiple machines within said cluster. This provides DE NIRO with more exibility to perform multiple, complex skills at the same time, culminating in a demonstration of a game of Hide and Seek using advanced computer vision and speech recognition.

Please find more information regarding the project in the "Final_Report.pdf" file.

## Instructions on Running The Code
The Robot Brain Cluster code base is designed to facilitate our "Hide and Seek" demonstration which uses the specific DE NIRO hardware. The ROS remote launch and Brain Cluster files also had to be tailored to some degree to the specific desktops we used in the cluster. Therefore, our full code is not expected to run without access to original setup.

It should also be noted that the object (YOLO), face (dlib) and particular (find_object_2D) nodes require external libraries to be installed alongside.

However, the code base was designed to make each component as general as possible. This means the ROS packages for object, face and particular detection should all be applicable to any ROS system that publishes image data with only small modifications. Equally, many elements of the UI such as hot word detection, instruction encoding and display could all be reused for other projects on DE NIRO or a robot with similar hardware. 

Once set up correctly, the entire system runs with a simple call `roslaunch state_machine state_machine.launch`.

## Demonstration

[![Watch the video]()](https://www.youtube.com/watch?v=yGSAi70YS4k&feature=youtu.be)
