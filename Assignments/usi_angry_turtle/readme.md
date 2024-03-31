# USI ANGRY TURTLE - Elvi Mihai Sabau Sabau
Assignament 1 for the Robotis Assignment at USI MSc in AI.

Link to video demo: https://youtu.be/MapncC5iA1w

To run the system, you need to have ros2 installed, and the turtlesim package installed.
Then, install this package.

Then run the following commands, one on each terminal.
    
```bash
ros2 run turtlesim turtlesim_node
```

```bash
ros2 run usi_angry_turtle offenders
```

```bash
ros2 run usi_angry_turtle writer
```

The writer node controls the turtle, and the offenders node manages the offender turtles.

To change the writting coordinates, speed, k1, k2, lookahead of the writer turtle, you can change the values in the writer_process file. The writer node is a 3 state state machine, switching between normal, angry and move back behaviours, and a lookahead mechanism to intercept the offenders.

To change the number of concurrent offenders, you can change the value 'concurrent_turtles' in the offenders_process file.