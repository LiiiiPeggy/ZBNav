# Map Downsample Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline C++ tool `map_downsample` to `odin_ros_driver` that reads the Odin prebuilt map `SLAM/src/odin_ros_driver/map/map_20260807_151455.ply`, voxel-downsamples it with `pcl::VoxelGrid<pcl::PointXYZ>` (leaf_size default 0.05 m), and writes `map_20260807_151455.pcd` — the first step toward publishing a map topic for RViz display + waypoint placement.

**Architecture:** A single pure command-line executable (no rclcpp / no ROS node) in `odin_ros_driver` that uses PCL's PLY reader, VoxelGrid filter, and binary PCD writer. The load+voxel logic stays in `main()` so the future topic publisher node can reuse the same PCL calls. The tool is registered in the package's existing ROS2 `install(TARGETS ...)` list so the binary actually lands in `install/.../lib/odin_ros_driver/`.

**Tech Stack:** C++17 (set at `CMakeLists.txt:96`), PCL (`pcl::io::loadPLYFile`, `pcl::VoxelGrid`, `pcl::io::savePCDFileBinary`, `pcl::PointXYZ`), `std::filesystem` for output size.

## Global Constraints

- Branch: `mapmulti`.
- C++17 (`set(CMAKE_CXX_STANDARD 17)` at CMakeLists.txt:96) — `std::filesystem` is available.
- **MUST add `map_downsample` to the existing ROS2 `install(TARGETS ...)` block** (the one ending in `RUNTIME DESTINATION lib/${PROJECT_NAME}`) — otherwise `colcon build` succeeds but `./install/odin_ros_driver/lib/odin_ros_driver/map_downsample` does not exist.
- **Do NOT add `${PCL_INCLUDE_DIRS}` per-target** — `find_package(PCL REQUIRED)` and `include_directories(... ${PCL_INCLUDE_DIRS} ...)` are already global; only `target_link_libraries(map_downsample ${PCL_LIBRARIES})` is needed.
- Point type is `pcl::PointXYZ` — the PLY has only `x y z` float fields (no color/intensity); do not fabricate intensity.
- `leaf_size` is the same for x/y/z, default `0.05` m.
- CLI: `map_downsample <input.ply> [output.pcd] [leaf_size]`; output defaults to `<input>` with `.ply` → `.pcd` in the same directory.
- Error handling: missing/unreadable PLY, empty cloud, or save failure → clear stderr message + non-zero exit; `leaf_size <= 0` → error + exit 1.
- `SLAM/src/odin_ros_driver/map/` is gitignored — the `.ply`, `.bin`, and generated `.pcd` must NOT be committed.
- Project marker rule: new/modified code blocks get a leading `// ################################` + `// C++: <description>` / `# CMake: <description>` marker; no END markers; markers must never alter behavior.
- Commit messages have NO `Co-Authored-By` trailer.
- Do NOT touch `host_sdk_sample`, the SLAM data path, Odin config, or any other package.

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `SLAM/src/odin_ros_driver/src/map_downsample.cpp` | Create | CLI parse + `loadPLYFile` + `VoxelGrid` + `savePCDFileBinary` + summary logs |
| `SLAM/src/odin_ros_driver/CMakeLists.txt` | Modify | add executable + link PCL + add `map_downsample` to the ROS2 `install(TARGETS ...)` list |

---

### Task 1: map_downsample tool (source + CMake + build + run)

**Files:**
- Create: `SLAM/src/odin_ros_driver/src/map_downsample.cpp`
- Modify: `SLAM/src/odin_ros_driver/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing (standalone CLI tool)
- Produces: executable `map_downsample` installed at `install/odin_ros_driver/lib/odin_ros_driver/map_downsample`; running it on the real PLY produces `map_20260807_151455.pcd` in the map directory.

- [ ] **Step 1: Write the source file**

Create `SLAM/src/odin_ros_driver/src/map_downsample.cpp`:

```cpp
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// ################################
// C++: downsample Odin PLY map and save as PCD
// ################################
int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: map_downsample <input.ply> [output.pcd] [leaf_size]" << std::endl;
    std::cerr << "  output.pcd defaults to <input> with .ply -> .pcd" << std::endl;
    std::cerr << "  leaf_size defaults to 0.05 m (VoxelGrid, same for x/y/z)" << std::endl;
    return 1;
  }

  const std::string input_path = argv[1];
  std::string output_path = (argc >= 3) ? argv[2] : std::string();
  double leaf_size = (argc >= 4) ? std::stod(argv[3]) : 0.05;

  if (output_path.empty()) {
    output_path = input_path;
    if (output_path.size() >= 4 &&
        output_path.compare(output_path.size() - 4, 4, ".ply") == 0) {
      output_path.replace(output_path.size() - 4, 4, ".pcd");
    } else {
      output_path += ".pcd";
    }
  }

  if (leaf_size <= 0.0) {
    std::cerr << "Error: leaf_size must be > 0 (got " << leaf_size << ")" << std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_path, *cloud) < 0) {
    std::cerr << "Error: failed to load PLY: " << input_path << std::endl;
    return 1;
  }
  if (cloud->empty()) {
    std::cerr << "Error: PLY contains no points: " << input_path << std::endl;
    return 1;
  }

  auto t0 = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.filter(*down);
  auto t1 = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  if (pcl::io::savePCDFileBinary(output_path, *down) < 0) {
    std::cerr << "Error: failed to save PCD: " << output_path << std::endl;
    return 1;
  }

  std::error_code ec;
  auto out_bytes = std::filesystem::file_size(output_path, ec);
  std::cout << "[map_downsample] " << input_path << " (" << cloud->size() << " pts)"
            << " -> " << output_path << " (" << down->size() << " pts)"
            << ", leaf=" << leaf_size << " m, " << ms << " ms"
            << ", out_size=" << (ec ? 0 : out_bytes / (1024 * 1024)) << " MiB" << std::endl;
  return 0;
}
```

- [ ] **Step 2: CMakeLists — add executable + link**

In `SLAM/src/odin_ros_driver/CMakeLists.txt`, immediately AFTER the `registered_scan_adapter_node` block (its `ament_target_dependencies(registered_scan_adapter_node ...)` ends right before the `# Installation rules` comment), add:

```cmake
    # ################################
    # CMake: add map_downsample offline PLY->PCD tool
    # ################################
    add_executable(map_downsample
        src/map_downsample.cpp
    )
    target_link_libraries(map_downsample
        ${PCL_LIBRARIES}
    )
```

Do NOT add `${PCL_INCLUDE_DIRS}` here — it is already global via `include_directories(...)`.

- [ ] **Step 3: CMakeLists — add to install(TARGETS ...)**

In the same file, in the existing `install(TARGETS ...)` block (currently lists `host_sdk_sample`, `pcd2depth_ros2_node`, `cloud_reprojection_ros2_node`, `image_overlay_node`, `registered_scan_adapter_node`, `depth_image_ros2_node_lib`, `pointcloud_depth_converter_ros2`, `cloud_reprojector_ros2`), add `map_downsample` after `registered_scan_adapter_node`:

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

Without this step the binary is compiled but not installed; the run command below would fail with "No such file or directory".

- [ ] **Step 4: Build**

```bash
cd /home/yu/Codes_rk/SLAM && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select odin_ros_driver
```

Expected: builds clean (pre-existing warnings OK); `install/odin_ros_driver/lib/odin_ros_driver/map_downsample` exists afterward.

- [ ] **Step 5: Run on the real map**

```bash
cd /home/yu/Codes_rk/SLAM
./install/odin_ros_driver/lib/odin_ros_driver/map_downsample \
  src/odin_ros_driver/map/map_20260807_151455.ply
```

Expected stdout like:
```
[map_downsample] .../map_20260807_151455.ply (2002452 pts) -> .../map_20260807_151455.pcd (<N> pts), leaf=0.05 m, ... ms, out_size=... MiB
```
with `N` much smaller than 2,002,452, and the file `src/odin_ros_driver/map/map_20260807_151455.pcd` exists.

- [ ] **Step 6: Error paths**

```bash
./install/odin_ros_driver/lib/odin_ros_driver/map_downsample /nonexistent.ply; echo "exit=$?"
```
Expected: `Error: failed to load PLY: /nonexistent.ply` on stderr and `exit=1`.

```bash
./install/odin_ros_driver/lib/odin_ros_driver/map_downsample src/odin_ros_driver/map/map_20260807_151455.ply /tmp/out.pcd 0
```
Expected: `Error: leaf_size must be > 0` and `exit=1`; `/tmp/out.pcd` NOT created.

- [ ] **Step 7: Confirm no repo pollution**

```bash
git -C /home/yu/Codes_rk status --short
```
Expected: the generated `.pcd` does NOT appear (it lives in the gitignored `map/`). Only the two files from this task are staged for the commit.

- [ ] **Step 8: Commit**

```bash
cd /home/yu/Codes_rk && git add SLAM/src/odin_ros_driver/src/map_downsample.cpp SLAM/src/odin_ros_driver/CMakeLists.txt
git commit -m "feat(odin): add map_downsample offline PLY->VoxelGrid->PCD tool"
```

No `Co-Authored-By` trailer.

---

## Verification (end-to-end)

1. Build succeeds and `./install/odin_ros_driver/lib/odin_ros_driver/map_downsample` exists (proves install(TARGETS) wiring).
2. `map_downsample src/odin_ros_driver/map/map_20260807_151455.ply` produces `map_20260807_151455.pcd`, summary shows input 2,002,452 → output count a small fraction of that, valid PCD binary.
3. Error paths exit non-zero with clear messages.
4. `git status` clean of the generated `.pcd`.

## Out of Scope

- Publishing the PCD as a PointCloud2 topic (future; reuses the same PCL load logic).
- RViz display config for the map cloud.
- Map-mode cruise (`multi_frame=map`) end-to-end integration.
