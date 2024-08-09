# manipulator_control_pkg

Message [Nate Adkins](mailto:npa00003@mix.wvu.edu) on Slack with any questions or suggestions

## Overview

This ROS 2 package provides nodes for controlling the manipulator.

## Nodes

### Node: N/A

- N/A

#### Parameters

- N/A

#### Action

- N/A


## Building and Running

1. Clone this package into your ROS 2 workspace.

    ```bash
    cd <path_to_your_workspace>
    cd src
    git clone git@github.com:wvu-urc/manipulator_control_pkg.git
    ```

2. Build the ROS 2 workspace.

    ```bash
    cd ..
    colcon build
    ```

3. Source the ROS 2 workspace.

    ```bash
    source setup.bash
    ```

4. Code can be ran once the nodes exist

## Known Limitations
Force feedback is not used which can result in motors overtorquing or breaking the arm if the operator is not paying attention

The CAN control node was used and does work. The UART control node was used on a prior iteration of the arm and may have some issues with control, but the ideas behind how it works have been proven to function

The Arm has some strange jumping behavior every once in a while where a motor suddenly moves an high power and causes the arm to break. The issue causing this is unknown, but it may be a deadband issue.

This should be used with the motor protection package as this package does not natively support command timeout and the arm can continue to travel if disconnected