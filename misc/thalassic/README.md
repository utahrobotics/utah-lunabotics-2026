# Thalassic

Contains Compute shader pipelines for processing point clouds.

## Occupancy Grid Pipeline
Populates an occupancy grid the size of the arena, where each cell is scored from 1-100 based on how occupied it appears to be (0 is considered unknown).

### Inputs
1. A pointcloud
2. The isometry of the camera.
3. The size of each cell in the grid.
4. neighborhood_radius: a configurable parameter to decide how many neighbors each cell is compared against to detect obstacles.
5. min_points_for_occupied: The number of points that have to have to fall in a cell for it to be considered to be known
6. min_known_neighbors_ratio: A percentage n such that if greater than n % of cells in a cell C's neighborhood are unknown, C is also unknown.
7. obstacle_threshold: Any cell with a score greater than this threshold will be considered an obstacle.

## Output
A grid the size of the arena where each cell has a value from 0 to 100, where 0 is considered unknown, and 1-100 are a score of how "occupied" a cell appears to be.

## Depth Projector
Generates point clouds from an array of depths.
### Inputs
1. A depth frame and associated metadata from a realsense camera.
2. The isometry of the depth camera.
3. A stride parameter used for downsampling the cloud.

### Outputs
1. A pointcloud.
