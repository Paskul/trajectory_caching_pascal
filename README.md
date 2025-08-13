# Trajectory caching workflow -- Pascal

## Launching

**TODO- make config section for voxel and update the readme accordingly for voxelization**


## Voxelizing
A voxel cloud represented as a point cloud topic (voxel midpoints) is automatically generated based on set user parameters, such as radius, height, size, etc. A point cloud is generated every second. **A voxel-map CSV is written to the current ROS workspace root as `voxel_map.csv`, hardcoded**. This saved voxel_map allows for easy lookup of voxel IDs corresponding to a (x,y,z) point in the base_link frame, needed for caching and later-execution. 

Current parameters set (correlating with .csv files in parent dir) include:
| Parameter | Value |
| ------------- | ------------- |
| `voxel_size`  | 0.15 (m) |
| `radius_max`  | 1.0 (m) |
| `z_min`  | 0.0 (m) |
| `z_max`  | 1.2 (m) |
| `cut_angle`  | -18.435 (deg) |
| `push_from_base`  | 0.6 (m) |


## All points caching
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

all_points_cacher parameters: **TODO, ACTUALLY MAKE THESE PARAMS AGAIN IN CONFIG**
| Parameter | Value |
| ------------- | ------------- |
| `caches_out_path`  | `/home/pascal/ros2_ws/refined_map.csv` |
| `z_max`  | 1.2 (m) |
| `cut_angle`  | -18.435 (deg) |
| `push_from_base`  | 0.6 (m) |


**WARNING** If an *all points CSV* creation was initially successful, and for any reason it was to be run again in post, **the initial CSV will be overwritten, and the initial cache will be deleted**. For this reason, it's highly recommended that after a cache is fully solved, it be duplicated/backed up in a separate directory.

## All points executing
**TODO ADD Notes on how all point exec works, pulling from CSV, checks with moveit, then exec**

## Hybrid execution (Early version of Hybrid planning)
**TODO: Mention what code currently does, how it's kind of broken, maybe it works from a starting point, but not for relating to everything else I've done**

## Apple Predictions (with visual in voxel)
Included in this package isn't just trajectory caching, but a sample integrated version of apple mask prediction via YOLO. Model weights are pulled from `/models` (in our case, `/models/best-merged-apples-thlo-merged-wsu-v8n.pt`) and used in `apple_pred.py`. Necessary ROS2 image topics are loaded and fed to the model, where at any instance, a colored image is used for classification, and its corresponding depth image is projected to map with the colored image, allowing us to pull the exact apple depth at the median of a mask.

We not only do this, but then publish a pointcloud topic **`apple_cloud` this needs to change to not be hardcoded, config add**, where said median points from any classified apple mask are republished in its own topic -- used for fitting which voxels have a genuine apple within them.

apple_pred parameters:
| Paramter  | Default |
| ------------- | ------------- |
| `color_topic`  | `/camera/color/image_rect` |
| `depth_topic`  | `/camera/depth/image_rect` |
| `info_topic`  | `/camera/color/camera_info_rect` |
| `model_path`  | `/home/pascal/ros2_ws/src/pascal_full/models/best-merged-apples-thlo-merged-wsu-v8n.pt` |
| `conf`  | 0.4 |
| `iou`  | 0.6 |
**TODO ADD A TOPIC FOR PUBLISHED APPLE CLOUD NAME**

With a published point cloud, we then call on `visualize_apple_pred.py` to visualize/ID each point cloud to its correct voxel. A ROS2 `MarkerArray` is used to publish cube markers alongside a point cloud, though this `MarkerArray` is really best used for visualization in practice. `visualize_apple_pred.py` can read **TODO ADD APPLE CLOUD TOPIC HERE** with each pointcloud's position in frame, iterate through voxels to find if that tested point lies within any voxel, and if so, publishes a deep red voxel in its place on the `MarkerArray` `apple_pred_marker_arrays` topic **TODO, MAKE A CUSTOM TOPIC NAME HERE**.

visualize_apple_pred parameters:
| Paramter  | Default |
| ------------- | ------------- |
| `apple_pointcloud_topic` NOT MADE YET | `/apple_cloud` |
| `apple_markerarray_topic` NOT MADE YET | `/apple_pred_marker_arrays` |
**TODO ADD A TOPIC FOR PUBLISHED APPLE CLOUD NAME**


## Misc. Notes
Caching is currently configured for the ur5e arm with starting joint-states at the `test` position found in the `apple-harvest` repo. Joint positions are conservative estimates, and are should be/are planned with some tolerance (as acceptance for MoveIt) on the `ur_manipulator` group name (this is important for caching/execution). As of writing, starting joint-state configurations are (and are hardcoded as accepting):
| Joint  | Position |
| ------------- | ------------- |
| `shoulder_pan_joint`  | 1.54 |
| `shoulder_lift_joint`  | -1.62 |
| `elbow_joint`  | 1.40 |
| `wrist_1_joint`  | -1.20 |
| `wrist_2_joint`  | -1.60 |
| `wrist_3_joint`  | -0.11 |

