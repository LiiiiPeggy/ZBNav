# Map Downsample: PLY → VoxelGrid → PCD

## Goal

An offline C++ tool `map_downsample` in `odin_ros_driver` that reads the Odin prebuilt map
`SLAM/src/odin_ros_driver/map/map_20260807_151455.ply` (binary little-endian, fields `x y z`
as float, 2,002,452 vertices), voxel-downsamples it with `pcl::VoxelGrid<pcl::PointXYZ>`
(leaf_size default **0.05 m**, adjustable), and writes `map_20260807_151455.pcd`.

This is step 1 of map-mode navigation: the downsampled `.pcd` will later be loaded by a ROS
node and published as a PointCloud2 topic for RViz display + waypoint placement. This step
only produces the `.pcd`; the topic publisher is out of scope.

## Background

- The `.bin` map is Odin-device-private; the SDK offers no "return loaded map cloud" API.
  A `.ply` export of the same map exists and is parseable by PCL (`pcl::io::loadPLYFile`).
- The PLY has only XYZ (no color / intensity) — the map is geometric only.
- `map/` is gitignored; the `.ply`, `.bin`, and future `.pcd` stay out of the repo.
- `odin_ros_driver/CMakeLists.txt` already does `find_package(PCL REQUIRED)` and links
  `${PCL_LIBRARIES}` for existing executables (`pcd2depth_node`, `cloud_reprojection_node`, …).

## Tool Interface

Pure command-line executable (no rclcpp / no ROS node):

```
map_downsample <input.ply> [output.pcd] [leaf_size]
```

- `input.ply` (required): source PLY file.
- `output.pcd` (optional): output path; defaults to `input` with `.ply` → `.pcd` in the same directory.
- `leaf_size` (optional): VoxelGrid leaf edge in meters, same for x/y/z; defaults to `0.05`.

## Behavior

1. `pcl::io::loadPLYFile(input, cloud)` into `pcl::PointCloud<pcl::PointXYZ>`.
   - On failure (missing file, unreadable header, empty cloud): print a clear error to stderr,
     exit non-zero. Empty cloud is an error, not a silent success.
2. `pcl::VoxelGrid<pcl::PointXYZ>` with `leaf_x = leaf_y = leaf_z = leaf_size`.
3. `pcl::io::savePCDFileBinary(output, downsampled)`.
4. Print summary to stdout:
   - input point count → output point count
   - elapsed time
   - output file size
5. Exit 0 on success, non-zero on any failure.

## Files

| File | Action | Responsibility |
|------|--------|----------------|
| `SLAM/src/odin_ros_driver/src/map_downsample.cpp` | Create | CLI arg parse + loadPLYFile + VoxelGrid + savePCDFileBinary + summary logs |
| `SLAM/src/odin_ros_driver/CMakeLists.txt` | Modify | add executable + link PCL + add `map_downsample` to the ROS2 `install(TARGETS …)` list (see CMake changes below) |

The load+VoxelGrid logic stays in one small function so the future topic publisher node can
reuse the same PCL calls (load PCD → optional re-downsample → publish).

## CMake changes (exact)

`find_package(PCL REQUIRED)` and `include_directories(... ${PCL_INCLUDE_DIRS} ...)` are already
global in the file — do NOT add `PCL_INCLUDE_DIRS` per-target. Only:

```cmake
add_executable(map_downsample
    src/map_downsample.cpp
)

target_link_libraries(map_downsample
    ${PCL_LIBRARIES}
)
```

**Required:** add `map_downsample` to the existing ROS2 install list (the block that ends with
`RUNTIME DESTINATION lib/${PROJECT_NAME}`), alongside `host_sdk_sample`, `pcd2depth_ros2_node`,
`cloud_reprojection_ros2_node`, `image_overlay_node`, `registered_scan_adapter_node`, …:

```cmake
install(TARGETS
    host_sdk_sample
    pcd2depth_ros2_node
    cloud_reprojection_ros2_node
    image_overlay_node
    registered_scan_adapter_node
    map_downsample
    depth_image_ros2_node_lib
    pointcloud_depth_converter_ros2
    cloud_reprojector_ros2
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

Without this, `colcon build` succeeds but
`./install/odin_ros_driver/lib/odin_ros_driver/map_downsample` would not exist.

## Verification

```bash
cd SLAM && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select odin_ros_driver

./install/odin_ros_driver/lib/odin_ros_driver/map_downsample \
  src/odin_ros_driver/map/map_20260807_151455.ply
```

Expected:
- Builds clean (only pre-existing warnings).
- Produces `src/odin_ros_driver/map/map_20260807_151455.pcd`.
- Output point count is a small fraction of the input 2,002,452 (0.05 m voxels).
- PCD header is valid (verify with `pcl::io::loadPCDFile` in a quick check or `pcl_viewer` if available).
- Non-zero exit + clear error for a nonexistent input path.

## Out of Scope

- Publishing the PCD as a PointCloud2 topic (future step; reuses this tool's PCL logic).
- RViz display config for the map cloud.
- Map-mode cruise (`multi_frame=map`) end-to-end integration.
- Anything about `/cruise_autonomy`, robot dynamics, or Odin relocalization configuration.
