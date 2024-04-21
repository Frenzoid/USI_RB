# USI THYMIO - Elvi Mihai Sabau Sabau & Yassine Oueslati
Assignament 2 for the Robotis Assignment at USI MSc in AI.
<hr />

## Task 1:
Write an open-loop controller that moves the MyT in such a way that it follows an "8" trajectory. Test it in the default empty scene.
### Solution:

For this task, we developed a controller which controls the thymio using 2 states, `circle_right` and `circle_left`. A timer of 5.5 seconds changes the trajectory, this is the exact time needed to complete a circle. The controller is implemented in a way that the robot will follow the 8 trajectory indefinitely. 

### Launch command:

```bash
ros2 launch usi_thymio eight.launch.xml thymio_name:=thymio_0
---------------------------------------
parameters:
  thymio_name: by default is thymio0, depending on which version of the bridge youre running, you might need to change it to thymio_0.
```

### Video: 
https://youtu.be/WXkSSgUwkO4?si=gl776xxTJmN6oOQN&t=5
<hr />

## Tasks 2 and 3:

Using the wall scene file, write a controller to move the MyT straight ahead. Note that this scene rotates randomly the wall every time it is reset. We assume the robot is heading toward a wall somewhere in front of it; the wall is not necessarily orthogonal to the direction the MyT is originally pointing to. Write the controller in such a way that the MyT moves straight ahead until it is close to the wall (without hitting it), then turns in place in such a way to face the wall as precisely as possible (i.e., the robot's x-axis should be orthogonal to the wall). To sense the wall orientation once you are close to it, you should use proximity sensors. Feel free to define a convenient distance threshold at which you decide to stop. (3pt)

Using the controller built for task 2, once the MyT arrives close to the wall, it should then turn in such a way that it is facing opposite to the wall, then move and stop in such a way that the robot is as close as possible to a point that is 2 meters away from the wall. Note that the proximity sensors don't have such a long range, so at some point, you'll have to rely on odometry. (4pt)

### Solution:

For this task we developed again a controller that uses 4 states: `FORWARD`, `FACE_WALL`, `TURN_AROUND` and `FORWARD_2M`.

A control loop handles the state transition, initially being `FORWARD`.

Logic of each state: 
- `FORWARD`: The robot moves forward until the proximity sensors detect an obstacle. To get a proper reading, the moment the sensors pick something up, the thymio will slow down and approach the wall until the sensor with the lowest value ( the closest sensor to the wall ) meets a certain distance threshold, close enough to the wall to get good readings from the sensors, but not enough to touch the wall. After this, the thymio stops and the state will change to `FACE_WALL`.

- `FACE_WALL`: The thymio will rotate towards the direction where the sensors have the highest value, this way we know we'll be rotating to align with the wall ( facing the wall ), and it will stop rotating the moment both central-lateral have the same value given a error threshold. After this, we stop moving the thymio the state will change to `TURN_AROUND`.

- `TURN_AROUND`: This one was the trickiest part, we rotate the thymio and also move slowly backwards ( we do this because if we rotate the thymio in place, the back sensors will still be too far away to detect the wall, and since odometry is screwed up beyond recognition we can't rely on anything else but sensors ) until any of the back sensors detect the wall. Then we stop rotating and we'll slowly move backwards and rotate towards the direction of the sensor with the highest value, this way we'll be aligning our back with the wall, and once both back sensors have the same value given an error threshold, we stop and change the state to `FORWARD_2M`.

- `FORWARD_2M`: We move slowly forward, and using the odometry ( coordinates )
 we calculate the euclidean distance traveled, and once we reach 2 meters, we stop the thymio and the controller.

```bash
ros2 launch usi_thymio compulsory.launch.xml thymio_name:=thymio_0
---------------------------------------
parameters:
  thymio_name: by default is thymio0, depending on which version of the bridge youre running, you might need to change it to thymio_0.
```

### Video: 
https://youtu.be/WXkSSgUwkO4?si=78hWAR4YBFo4-Iug&t=13

<hr />


## Advanced 1 and 2:

Write a controller that randomly explores an environment, avoiding any obstacles (similar to what a Roomba would do).
Deploy the robot in the scene that we made (link) for Coppelia.
Submit a link to a video demonstrating the robot's behavior (accelerate the video 10x to show in 10 seconds how the robot behaves over 100 simulated seconds)

Explore the environment as before, but simulate two or more robots at once. Adapt the controller so that the two Thymios avoid each other when they get close. Make sure your video shows what happens.
In case you complete this task, make advanced.launch.xml launch it one instead of advanced task 1

### Solution:

For this tasks we made a simple controller that moves forward, and when the proximity sensors detect an obstacle, the thymio will rotate toward the direction with the highest sensor value, and then move forward again. This way the thymio will avoid obstacles.

The controller also implements a squeeze-trough feature, the tymio will try and squeeze through tight spaces if no lateral sensor reaches certain closeness threshold and the central sensors detect no obstacles, this way the thymio will be able to explore the environment more efficiently.

Also in case of _uncertain conditions_*, the thymio will rotate towards a random direction, this will also avoid the thymio getting stuck in a _movement loop_**.

Also, given this previous logic, we dont need to implement anything else to make the thymio avoid each other, since the thymio will stop and rotate when it detects an obstacle.

*: uncertain conditions are when the thymio doesn't know if it should turn right or left, this happens for example when the thymio is in a corner and the sensors detect obstacles at the same dfistances on both sides.

**: movement loop is when the thymio is repeating the same movement pattern due to the environment layout.

### Launch command:

```bash
ros2 launch usi_thymio advanced.launch.xml thymio_0:=thymio_0 thymio_1:=thymio_1
---------------------------------------
parameters:
  thymio_0: name of the first thymio
  thymio_1: name of the second thymio
```

### Video ( Exploration ):
https://youtu.be/WXkSSgUwkO4?si=56n3iOkhu9C0v4nH&t=48
### Video ( Avoidance ):
https://youtu.be/WXkSSgUwkO4?si=FXbRLg13lhLpxZSU&t=98
