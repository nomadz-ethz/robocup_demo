# Change Log
## 2025.05.29
1. Change joystick subscription from /joy to /remote_controller_state, which is internal msg of Booster controller,
as /remote_controller_state handle different type of joystick internally, which means which type of joystick is
transparent to the brain_node, and thus no more need for joystick type.