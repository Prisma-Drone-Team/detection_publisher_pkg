# Detection Publisher Package

## Overview

The `detection_publisher_pkg` is a ROS 2 package that serves as a centralized fusion node for multi-modal detection data in the Leonardo autonomous system. It aggregates and republishes detection information from multiple sources including ArUco markers and YOLO object detection, providing enhanced accuracy through weighted averaging of multiple observations.

## Features

- **Multi-Source Detection Fusion**: Combines ArUco marker detections from both drone and rover platforms
- **YOLO Object Detection Integration**: Processes and spatially localizes objects detected by computer vision
- **Weighted Average Updates**: Implements intelligent fusion that improves accuracy with each new observation
- **Transform Management**: Publishes TF transforms for detected markers and objects in the global map frame
- **Duplicate Detection Handling**: Prevents redundant entries while continuously refining position estimates

## Architecture

### Core Components

1. **DetectionPublisher Node**: Main ROS 2 node handling all detection streams
2. **Detection Structures**: Custom data structures for storing detection metadata
3. **Transform Broadcasters**: Multiple TF broadcasters for different detection types
4. **Weighted Fusion Algorithm**: Mathematical fusion of repeated observations

### Data Structures

```cpp
struct Detection {
    char robot;                                     // Robot ID ('D' for drone)
    int marker_id;                                  // ArUco marker identifier
    geometry_msgs::msg::TransformStamped tf_detection;  // 6DOF pose in map frame
    int n_observations;                             // Number of observations for this marker
};

struct ObjectDetection {
    char robot;                                     // Robot ID
    std::string object_class;                       // Object class from YOLO (e.g., "plant", "traffic_light")
    geometry_msgs::msg::TransformStamped tf_object; // 6DOF pose in map frame
    int n_observations;                             // Number of observations for this object
};
```

## Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/aruco_detector/tf_list` | `geometry_msgs::msg::TransformStamped` | ArUco marker detections from drone |
| `/rover_aruco_detector/tf_list` | `geometry_msgs::msg::TransformStamped` | ArUco marker detections from rover |
| `/yolo/prediction/item_dict` | `std_msgs::msg::String` | YOLO object detection results (JSON format) |

## Published Topics

### TF Transforms
The node publishes multiple TF transforms at 10 Hz:

- **Marker Transforms**: `map` → `marker_id<ID>` for each detected ArUco marker
- **Object Transforms**: `map` → `<object_class>` for each detected object
- **Target Transforms**: Additional transforms for navigation purposes:
  - `marker_id<ID>_rot` → `marker_id<ID>.target` (1m offset for approach)
  - `<object_class>` → `<object_class>.target` (1.65m lateral offset)

## Fusion Algorithm

### Weighted Average Update

When a marker or object is detected multiple times, the system updates its position using weighted averaging:

```cpp
// Calculate weights
double weight_old = n_observations / (n_observations + 1)
double weight_new = 1.0 / (n_observations + 1)

// Update position
new_position = old_position * weight_old + measured_position * weight_new
```

This approach:
- Gives higher weight to accumulated observations
- Reduces noise and improves accuracy over time
- Converges to stable position estimates

### Coordinate Frame Transformations

1. **Camera → Map**: Uses TF buffer to lookup current robot pose
2. **YOLO Processing**: Converts 2D image detections to 3D world coordinates
3. **ArUco Processing**: Direct 6DOF pose from marker detection

## Configuration

### Robot Identification
- Default robot ID: `'D'` (drone)
- Can be modified in constructor for multi-robot deployments

### Object Class Mapping
YOLO classes are converted to Leonardo-specific names:
```cpp
"potted_plant" → "plant"
"teddy_bear" → "teddy_bear" (unchanged)
"fire_hydrant" → "fire_hydrant" (unchanged)
```

### Distance Filtering
- Objects beyond 4.0m are filtered out for reliability
- Configurable via `z > 4.0` threshold in `parse_json_data()`

## Building and Running

### Dependencies
- ROS 2 (tested on Humble)
- tf2 and tf2_ros
- geometry_msgs
- std_msgs
- Eigen3
- nlohmann/json

### Build
```bash
cd leonardo_managers
colcon build --packages-select detection_publisher_pkg
source install/setup.bash
```

### Run
```bash
ros2 run detection_publisher_pkg detection_pub_node
```