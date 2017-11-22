### Overview
This folder holds source code for Kubo, the lab's autonomous tractor.

### Folders

#### camera

#### hemisphere

#### hind_brain
This folder contains Arduino code for Kubo's hind brain, running on an Arduino
Teensie. The code is run upon startup of Kubo's electronics; to interface with
the code, follow the tractor startup instructions on the home page of the Github
gravl wiki.

#### state
This folder contains C++ files for Kubo's basic auto/teleop state machine. This
code listens to the `/auto` topic and forwards data from either `/autodrive` or
`/teledrive` to `/drive` based on the boolean sent to `/auto`. To run the state
machine, ensure there is a roscore running, then in a new Terminal execute
`rosrun tractor State`.

#### teleop

#### waypoint
