# Trajectory caching workflow -- Pascal

## Launching

### Voxelization
To run voxelization of a workspace in front of the arm, run:
```bash
ros2 launch pascal_full voxelize_launch.py 
```
This will begin to publish a voxelized space in front of the arm, according to the parameters set. A `MarkerArray` message, as well as a `PointCloud2` topic will be published to encapsulate voxel information.

### Caching
To cache trajectories for the UR5e arm to each voxel midpoint, run:
```bash
ros2 launch pascal_full all_points_cache_launch.py 
```
**NOTE: This requires the voxelization node to be running.** It's recommended to have two separate terminals running, one for voxelization and one for caching. This launch encapsulates two nodes, one as a high-level 'helper' node to manage CSV and the entire voxel space, the 'all_points...' node. The other is for singular points, where calculations are done per-voxel.

### Execution
To execute trajectories for the UR5e arm to each voxel midpoint (from cache), run:
```bash
ros2 launch pascal_full all_points_execute_launch.py 
```
NOTE: This doesn't explicitly require, but wouldn't function without the voxelization node running. It's recommended to have two separate terminals running, one for voxelization and one for execution. This launch encapsulates two nodes, one as a high-level 'helper' node to load/publish segments of the CSV, the 'all_points...' node. The other is for singular trajectories, where trajectories are pulled from the CSV and attempted to execute by MoveIt.

### Apple Predictions
To run an apple classification YOLO model, really only for visualization of apples on voxelization, run:
```bash
ros2 launch pascal_full apple_launch.py 
```
This loads a YOLO model made for apple classification (masks) in RGB, where depth is applied and results in apples' localized to the camera in 3D space. Two nodes are run, one to handle actual apple predictions, and the other to visualize these predictions, as a `MarkerArray` topic publishing apple spaces as voxels to be applied as a layer to our current voxelized space.

### Hybrid planning/control
To run a demo of hybrid planning/control of the UR5e arm, run:
```bash
ros2 launch pascal_full hybrid_exec_launch.py 
```
This calls on a custom node which interacts with MoveIt to initiate a global planner, local planner, and hybrid manager to recognize collision objects and halt/adjust accordingly. Notes on further specifications are elaborated below.

## Parameters and uses

### Voxelizing
A voxel cloud represented as a point cloud topic (voxel midpoints) is automatically generated based on set user parameters, such as radius, height, size, etc. A point cloud is generated every second. A voxel-map CSV is written to the current `voxel_map_path` parameter as `voxel_map.csv`. This saved voxel_map allows for easy lookup of voxel IDs corresponding to a (x,y,z) point in the base_link frame, needed for caching and later execution. 

Current parameters set (correlating with .csv files in parent dir) include:
| Parameter | Default |
| ------------- | ------------- |
| `base_frame`  | base_link |
| `camera_frame`  | camera_link |
| `voxel_size`  | 0.15 |
| `radius_max`  | 1.0 |
| `z_min`  | 0.0 |
| `z_max`  | 1.2 |
| `cut_angle`  | -18.435 |
| `push_from_base`  | 0.6 |
| `voxel_markerarray_topic`  | workspace_voxels |
| `voxel_pointcloud_topic`  | workspace_voxel_centers |
| `apple_point_topic`  | apple_cloud |
| `voxel_map_path`  | /home/pascal/ros2_ws/voxel_map.csv |

### All points caching
Caching with this tool solves for highly-manipulable trajectories at two different approaches:
1. From `test` to `approach`
2. From `approach` to `center`

`test` is the starting joint-state as found in `harvest-repo`. Provided a voxelized workspace, this approach currently creates a cached trajectory for *each voxel* (represented as a pointcloud center).

`approach` is determined through an approach to maximize manipulability. For each voxel point generated, a spherical cone is projected from the original voxel point, outwards towards the `gripper_link` as described by the TF tree. The *surface intersection of the sphere with the cone* then has a uniform sampling to determine a smaller range of end-point poses that the robot arm will attempt to solve for. All uniformly selected subpoints of the cone's surface are tested as an endpoint from the starting pose.

Iteratively, a MoveIt **add more specifics of params here** RRTStar trajectory solving operation, each (could be final) surface subpoint from the starting test pose. This RRTStar solver utilizes a multi-objective optimization to find a trajectory with:

70% weight to solving for shorter path length
30% weight to solving for manipulability

This is done to avoid joint limits while solving for a genuinely desirable trajectory in the cache. After each point is attempted to be solved, an end-pose operation is given as:
score = (0.5)*manipulability_at_start_pose + (0.5)*manipulability_at_end_pose.

The point (and its associated trajectory) resulting in the strongest score is chosen and cached. **This is honestly somewhat futile, and could be greatly improved by considering manipulability along the entire trajectory, not just start/end poses. Especially when RRTStar is already working to introduce some manipulability.** The final saved trajectory would result in moving from `test` to `approach`.

Each voxel point has a generated, resulting in an `O(vcm)` time complexity for the caching operation from `test` to `approach`, where:
`v` represents the number of voxel points,
`c` represents the number of cone points, and
`m` represents the time complexity for each trajectory calculation (RRTStar) by MoveIt

`center` is the exact midpoint of a voxel. When caching, we understand this to be an estimate of the real apple's location, where we migrate from the approach pose to the center pose as 'approaching the apple'. It is expected that the `center` location itself is not actually the apple location itself, as a voxel's cache should be triggered **if the apple falls anywhere within voxel bounds**, typically not found in the exact center. For this reason alone, an additional custom *servo controller* must be implemented, but has not been done yet.

To estimate a similar result of `approach` to `center`, a Cartesian path seeded from the selected `test` to `approach` trajectory end-pose to voxel-midpoint (`center`) is calculated, and stored. This Cartesian path does not consider manipulability. I would expect it still avoids joint limits, but there is nothing proven here, and this is the exact reason a custom servo-controller is needed as a last step.

all_points_cacher (for all points, iterates over trajectory_cacher) parameters:
| Parameter | Default |
| ------------- | ------------- |
| `caches_out_path`  | /home/pascal/ros2_ws/cache_map.csv |
| `voxel_centers_topic`  | /workspace_voxel_centers |
| `cone_radius`  | 0.20 |
| `cone_num_pts`  | 20 |

trajectory_cacher (then called multiple times, used for one point) parameters:
| Parameter | Default |
| ------------- | ------------- |
| `approach_weight`  | 0.5 |
| `final_weight`  | 0.5 |
| `cone_angle`  | 45.0 |
| `group_name`  | ur_manipulator |
| `cone_points_topic`  | sampled_cone_points |



**WARNING** If an *all points CSV* creation was initially successful, and for any reason it was to be run again in post, **the initial CSV will be overwritten, and the initial cache will be deleted**. For this reason, it's highly recommended that after a cache is fully solved, it be duplicated/backed up in a separate directory.

### All points executing
**TODO ADD Notes on how all point exec works, pulling from CSV, checks with moveit, then exec**

all_points_execute (handles approach/final, point selection, sends plan) parameters:
| Parameter | Default |
| ------------- | ------------- |
| `voxel_map_path`  | /home/pascal/ros2_ws/voxel_map.csv |
| `cache_map`  | /home/pascal/ros2_ws/cache_map.csv |
| `segment`  | approach |
| `clicked_point_topic`  | /clicked_point |
| `trajectory_topic`  | /pascal_cached_trajectory |

trajectory_cacher (executes sent plan) parameters:
| Parameter | Default |
| ------------- | ------------- |
| `trajectory_topic`  | /pascal_cached_trajectory |


### Hybrid execution (Early version of Hybrid planning)
**TODO: Mention what code currently does, how it's kind of broken, maybe it works from a starting point, but not for relating to everything else I've done**


hybrid_exec (global/local planning and execute) parameters:
| Parameter | Default |
| ------------- | ------------- |
| `group_name`  | ur_manipulator |
| `pipeline_id`  | ompl |


### Apple Predictions (with visual in voxel)
Included in this package isn't just trajectory caching, but a sample integrated version of apple mask prediction via YOLO. Model weights are pulled from `/models` (in our case, `/models/best-merged-apples-thlo-merged-wsu-v8n.pt`) and used in `apple_pred.py`. Necessary ROS2 image topics are loaded and fed to the model, where at any instance, a colored image is used for classification, and its corresponding depth image is projected to map with the colored image, allowing us to pull the exact apple depth at the median of a mask.

We not only do this, but then publish a pointcloud topic **`apple_cloud` this needs to change to not be hardcoded, config add**, where said median points from any classified apple mask are republished in its own topic -- used for fitting which voxels have a genuine apple within them.

apple_pred parameters:
| Paramter  | Default |
| ------------- | ------------- |
| `color_topic`  | /camera/color/image_rect |
| `depth_topic`  | /camera/depth/image_rect |
| `info_topic`  | /camera/color/camera_info_rect |
| `model_path`  | /home/pascal/ros2_ws/src/pascal_full/models/best-merged-apples-thlo-merged-wsu-v8n.pt |
| `apple_points_topic`  | apple_cloud |
| `conf`  | 0.4 |
| `iou`  | 0.6 |

With a published point cloud, we then call on `visualize_apple_pred.py` to visualize/ID each point cloud to its correct voxel. A ROS2 `MarkerArray` is used to publish cube markers alongside a point cloud, though this `MarkerArray` is really best used for visualization in practice. `visualize_apple_pred.py` can read **TODO ADD APPLE CLOUD TOPIC HERE** with each pointcloud's position in frame, iterate through voxels to find if that tested point lies within any voxel, and if so, publishes a deep red voxel in its place on the `MarkerArray` `apple_pred_marker_arrays` topic **TODO, MAKE A CUSTOM TOPIC NAME HERE**.

### Visualize Apple Predictions
**TODO, that segmenet above could be moved down?**

visualize_apple_pred parameters:
| Paramter  | Default |
| ------------- | ------------- |
| `apple_points_topic` | apple_cloud |
| `apple_marker_arrays_topic` | apple_pred_marker_arrays |


## Misc. Notes
Caching is currently configured for the ur5e arm with starting joint-states at the `test` position found in the `apple-harvest` repo. Launching does not put you at the `test` position -- you must move there first. Joint positions are conservative estimates, and should be/are planned with some tolerance (as acceptance for MoveIt) on the `ur_manipulator` group name (this is important for caching/execution). As of writing, starting joint-state configurations are (and are hardcoded as accepting):
| Joint  | Position |
| ------------- | ------------- |
| `shoulder_pan_joint`  | 1.54 |
| `shoulder_lift_joint`  | -1.62 |
| `elbow_joint`  | 1.40 |
| `wrist_1_joint`  | -1.20 |
| `wrist_2_joint`  | -1.60 |
| `wrist_3_joint`  | -0.11 |

