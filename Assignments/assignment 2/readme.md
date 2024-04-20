# USI THYMIO - Elvi Mihai Sabau Sabau & Yassine Oueslati
Assignament 2 for the Robotis Assignment at USI MSc in AI.
<hr />

## Task 1:

### Solution:

### Launch command:

```bash

```

### Video: 
https://youtu.be/Jsk6gmVXEL0?si=huvTLN1Aj3gelefZ&t=5
<hr />

## Tasks 2 and 3:

Using the wall scene file, write a controller to move the MyT straight ahead. Note that this scene rotates randomly the wall every time it is reset. We assume the robot is heading toward a wall somewhere in front of it; the wall is not necessarily orthogonal to the direction the MyT is originally pointing to. Write the controller in such a way that the MyT moves straight ahead until it is close to the wall (without hitting it), then turns in place in such a way to face the wall as precisely as possible (i.e., the robot's x-axis should be orthogonal to the wall). To sense the wall orientation once you are close to it, you should use proximity sensors. Feel free to define a convenient distance threshold at which you decide to stop. (3pt)

Using the controller built for task 2, once the MyT arrives close to the wall, it should then turn in such a way that it is facing opposite to the wall, then move and stop in such a way that the robot is as close as possible to a point that is 2 meters away from the wall. Note that the proximity sensors don't have such a long range, so at some point, you'll have to rely on odometry. (4pt)

### Solution:


### Launch command:
```bash

```
### Video: 
https://youtu.be/Jsk6gmVXEL0?si=yc3ua-VoAy6k4dqJ&t=16

<hr />


## Advanced 1 and 2:

Write a controller that randomly explores an environment, avoiding any obstacles (similar to what a Roomba would do).
Deploy the robot in the scene that we made (link) for Coppelia.
Submit a link to a video demonstrating the robot's behavior (accelerate the video 10x to show in 10 seconds how the robot behaves over 100 simulated seconds)

Explore the environment as before, but simulate two or more robots at once. Adapt the controller so that the two Thymios avoid each other when they get close. Make sure your video shows what happens.
In case you complete this task, make advanced.launch.xml launch it one instead of advanced task 1

### Solution:



### Launch command:

```bash

```

### Video ( Exploration ):
https://youtu.be/Jsk6gmVXEL0?si=xP8W4LiJT88ntZBG
### Video ( Avoidance ):
https://youtu.be/Jsk6gmVXEL0?si=w4g2lT2r7kvBhBqB&t=145
