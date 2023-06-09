# PARC Engineering League

The University of Cape Verde - Smart Solutions Lab

### Members
- Sónia Semedo
- Anaxímeno Brito
- Kennedy Pina

## Solutions

### Task 1: Autonomous Field Navigation

For this task we used some of the sensors provided to determine the value of the rotation and the intensity of the rotation the robot. The system architecture was based on the classic structure of an intelligent system, having different parts like the perception (that receives all the relevant data published to the topics), the control (which we use to control the behavior of the robot), and the planning module (which is responsible by using the data gathered to create the best action to be taken by the robot), so that when you combine all those parts you will get an autonomous agent capable of (ideally) moving the proposed environment. For making the robot to move successfully the environment, lots of techniques from the areas of computer vision, mathematics, and basic artificial intelligence where used, which can be found on the libraries we created named **vision.py** and **ruler.py**.

#### Dependencies
Some of the dependencies needed to run our project can be found inside the package.xml file, therefore, can be easily installed by the ROS package manager you are using, but there will also be listed some the python libraries we used, but that are not listed in the package.xml file:

- OpenCV >= 4.2.0
- Numpy >= 1.17.4

#### How to Run?
Supposing you already have the system configured, the task 1 can be executed with the following steps:

1. Downloading and moving the source folder of our project to the `catkin_ws/src` dir, the going back to the root of the directory `catkin_ws` and running `catkin_make` which will build the code and download required packages.
2. Run the task number 1 recurring to the route you want to test our agent against
3. Execute how node by either:
a. Going through the folders until reaching the folder scripts of the folder task-1-solution and running the main script with `./main.py`
b. Using the rosrun command, giving the name of the package and the main script, being `rosrun task_1_solution main.py`
c. Executing recurring to the bash script inside the scripts folder by running `./retest.sh`. This scripts resets the position of the robot before running it to avoid having the robot in an inconsistent position, something that happened a lot while we were building and testing our project.
4. For finishing the node after completion of the route just hit `Crtl+c` in the terminal the node is running or send and interrupt signal with the following command `rosnode kill /ucv_task_1_solver_robot_agent`


### Task 2: Weed Detection

The implementation strategy outlined was that the image processing would start as soon as the robot's state was started and stop with the finished state.
However, subscribing to the topic robot_status has been showing irregular behaviour in matlab.
The topic parc_robot/weed_detection doesn't appear either, so I have a return in the code
For the processing we decided to subscribe to the topics gps, right and left cameras.
For each interaction we collect information from these topics:

- The GPS coordinates are converted to Cartesian

- The images after processing are stored and the centroid positions of the detected herbs are saved;
```matlab
centroidsRight = cat(1,centroidsRight,stats.Centroid)
```

- The centroid pixels are converted into Cartesian coordinates
```mathlab
% pixel to x,y
pixel_x=centroids(:,1);
pixel_y=centroids(:,2);

x = (pixel_x - image_width / 2) * (2 / image_width) * tan(hFov / 2)
y = (pixel_y - image_height / 2) * (2 / image_height) * tan(vFov / 2)

```

- These Cartesian coordinates would then be converted through the optical_camera to base_link transformation matrix
```matlab
mountToOdom = getTransform(tftree, 'base_link', 'zed2_right_camera_optical_frame');
mountToOdomTranslation = mountToOdom.Transform.Translation
mountToOdomRotation = mountToOdom.Transform.Rotation
mountToOdomRotationAngles = rad2deg(quat2eul([mountToOdomRotation.W mountToOdomRotation.X mountToOdomRotation.Y mountToOdomRotation.Z]))
```
- The results would be saved in two vectors for later publication in the weed detection
```matlab
Weed_detection = cat(1, Weed_detection, Weed_detection)
```

#### How to run?
Just open the correspondent matlab files in matlab and run it altogether with the task 2. 
## Challenges faced
We've faced many challenges during the project, but we overcame some of them.

- The first challenge was the nature of the tasks, which is something most of the elements (if not all) of the group were not accustomed working with

- Another challenge was to define which of the sensors were the most adequate each of the problems we had to solve, to overcome that we had to test many of the sensors and stick with the ones the gave better results to our tests

- The time we had to complete the tasks didn't help, even more considering that we had to learn ROS from the beginning, but, we have the sense that working with time restriction is also a fundamental part of engineering

- Encountering the GPS data on the task 2 was a considerable challenge as well

## More info
The original repository by which this project was run (even though not exclusively) was [anaximeno/parc-unicv](https://github.com/anaximeno/parc-unicv.git).