# Change Log
## 2025.06.19
1. Add kick demo, only for T1 with 4dof arm, motion firmware should be greater than 1.2.0.9, and you should run internal sdk installer
 in the doc https://booster.feishu.cn/wiki/CKCAwl2hCisDI2k4BRTcimi2nGb for successful compiling robocup_demo. Vision calibration is required for this kick action need ball pos on robot frame.
```bash
./scripts/start.sh tree:=test_kick
```
## 2025.05.30
1. Add Subtree find ball, which is a demo of how to find ball with moving, fix the problem of if not find ball, robot will not move.
## 2025.05.29
1. Change joystick subscription from /joy to /remote_controller_state, which is internal msg of Booster controller,
as /remote_controller_state handle different type of joystick internally, which means which type of joystick is
transparent to the brain_node, and thus no more need for joystick type.