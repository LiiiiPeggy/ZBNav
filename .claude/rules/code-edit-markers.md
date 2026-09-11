# Code Modification Comment Rules

Whenever adding or modifying source code, add a clear comment marker immediately BEFORE each logically modified code block.

Do NOT add any END marker after the modified code.

## Required format

Use exactly this structure:

```text
<comment> ################################
<comment> <Language>: <short modification description>
<comment> ################################

modified code
```

The comment syntax MUST match the programming language or file format being edited.

The description should briefly explain what the following modified code does.

Do NOT use generic descriptions such as:

```text
modified code
new code
change
update
fix
```

Prefer functional descriptions such as:

```text
reject obstacles near clear centers
add gate/bridge obstacle
handle repeat waypoint mode
filter near-range point cloud
publish registered scan
check destination reached
```

## C / C++ / CUDA

Use `//`.

Example:

```cpp
// ################################
// C++: reject obstacles near clear centers
// ################################
bool InClearZone(double x, double y) {
    if (_clear_radius <= 0.0 || _clear_centers.empty()) return false;

    Eigen::Vector2d p(x, y);
    for (const auto &c : _clear_centers) {
        if ((p - c).norm() < _clear_radius) return true;
    }

    return false;
}

// ################################
// C++: add gate/bridge obstacle
// ################################
void AddBridge(
    const Eigen::Vector2d &pos_in,
    const Eigen::Vector3d &bridge_size,
    pcl::PointCloud<pcl::PointXYZ> &cloudMap) {

    // implementation
}
```

## Python

Use `#`.

```python
# ################################
# Python: filter invalid lidar points
# ################################
valid_points = [
    p for p in points
    if p.distance >= min_range
]
```

## Bash / Shell

Use `#`.

```bash
# ################################
# Bash: configure ROS environment
# ################################
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## YAML

Use `#`.

```yaml
# ################################
# YAML: configure repeat cruise mode
# ################################
repeat_mode: true
repeat_count: -1
```

## CMake

Use `#`.

```cmake
# ################################
# CMake: add PCL dependency
# ################################
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
```

## XML / HTML / URDF / ROS XML Launch

Use XML comments.

```xml
<!-- ################################ -->
<!-- XML: add lidar static transform -->
<!-- ################################ -->
<node pkg="tf2_ros"
      type="static_transform_publisher"
      name="lidar_tf"
      args="0 0 0.2 0 0 0 base_link lidar_link"/>
```

## Java / JavaScript / TypeScript

Use `//`.

```javascript
// ################################
// JavaScript: validate navigation goal
// ################################
if (!goal || !goal.position) {
    return false;
}
```

## CSS

Use `/* */`.

```css
/* ################################ */
/* CSS: adjust navigation panel width */
/* ################################ */
.navigation-panel {
    width: 320px;
}
```

## MATLAB

Use `%`.

```matlab
% ################################
% MATLAB: remove invalid trajectory samples
% ################################
trajectory = trajectory(valid_indices, :);
```

## Lua / SQL

Use `--`.

```lua
-- ################################
-- Lua: initialize robot state
-- ################################
robot_state = "IDLE"
```

# Mandatory Rules

1. Add the marker only BEFORE newly added or actually modified logic.

2. Do NOT add any END marker after the modified code.

3. Do NOT add comments such as:

```text
end
modified code end
change end
################################
```

after the modified block.

4. Each independent logical modification should have its own marker.

Example:

```cpp
// ################################
// C++: add clear-zone collision check
// ################################
bool InClearZone(...) {
    ...
}

existing unchanged code...

// ################################
// C++: add bridge obstacle generation
// ################################
void AddBridge(...) {
    ...
}
```

5. If several adjacent changes implement the same feature, use only one marker before the complete logical block.

6. Do NOT place a marker before every modified line.

Incorrect:

```cpp
// ################################
// C++: initialize repeat mode
// ################################
repeat_mode_ = true;

// ################################
// C++: initialize repeat count
// ################################
repeat_count_ = 0;
```

Prefer:

```cpp
// ################################
// C++: initialize repeat cruise state
// ################################
repeat_mode_ = true;
repeat_count_ = 0;
returning_to_start_ = false;
```

7. Use the actual programming language or file type in the second comment line.

Examples:

```text
C++
Python
Bash
YAML
CMake
XML
JavaScript
TypeScript
MATLAB
Lua
SQL
```

8. Keep modification descriptions concise, preferably 3-10 words.

9. Describe the functional purpose of the modification rather than the editing operation.

Prefer:

```text
C++: reject obstacles near clear centers
C++: add gate/bridge obstacle
C++: implement repeat cruise return
Python: filter near-range lidar points
YAML: configure planner collision threshold
```

Avoid:

```text
C++: modify code
C++: fix function
C++: update implementation
C++: add new code
```

10. Existing modification markers MUST NOT be removed unless explicitly requested.

11. If code immediately following an existing marker is modified again and still serves the same logical purpose, reuse the existing marker instead of adding another marker.

12. If the purpose of the block changes substantially, update the existing marker description so it accurately describes the current logic.

13. Do not preserve obsolete implementations by commenting them out unless explicitly requested.

14. Modification markers must never alter runtime behavior.

15. For strict JSON files, comments are invalid. Do NOT insert modification markers into `.json` files. Modify the JSON normally.

16. Apply these rules to all code modifications, including:

* feature additions
* bug fixes
* refactoring
* parameter changes
* ROS nodes
* ROS launch files
* configuration files
* CMake changes
* shell scripts
* simulation code
* planner modifications
* controller modifications

17. When presenting modified code in the response, preserve the same modification markers that were inserted into the actual source files.

18. Always add these markers automatically whenever modifying code. The user should not need to request them again.