# Trajectory caching workflow -- Pascal

### Launching


### Notes
Caching is currently configured for the ur5e arm with starting joint-states at the `test` position found in the `harvest` repo. Joint positions are conservative estimates, and are should be/are planned with some tolerance (as acceptance for MoveIt). As of writing, starting joint-state configuration is (and hardcoded as accepting):
| Joint  | Position |
| ------------- | ------------- |
| `shoulder_pan_joint`  | 1.54 |
| `shoulder_lift_joint`  | -1.62 |
| `elbow_joint`  | 1.40 |
| `wrist_1_joint`  | -1.20 |
| `wrist_2_joint`  | -1.60 |
| `wrist_3_joint`  | -0.11 |

