# Change Log
## 2025.07.08
1. Support jetpack 6.2, add model for jetpack 6.2
## 2025.07.04
1. Add backward-ros for more convenient debugging
## 2025.05.30
1. Add Subtree find ball, which is a demo of how to find ball with moving, fix the problem of if not find ball, robot
will not move
## 2025.05.29
1. Change joystick subscription from /joy to /remote_controller_state, which is internal msg of Booster controller,
as /remote_controller_state handle different type of joystick internally, which means which type of joystick is
transparent to the brain_node, and thus no more need for joystick type.