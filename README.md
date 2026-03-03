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
5. **Multi-Robot Cross-Platform Fusion**: Single detection vector for unified drone+rover observations
6. **Timer-Based Publishers**: 10 Hz TF publication for real-time navigation

### Data Structures

```cpp
struct Detection {
    char robot;                                     // Robot ID ('D'=drone, 'R'=rover, 'M'=multi-robot)
    int marker_id;                                  // ArUco marker identifier
    geometry_msgs::msg::TransformStamped tf_detection;  // 6DOF pose in map frame
    int n_observations;                             // Number of observations for this marker
};

struct ObjectDetection {
    char robot;                                     // Robot ID ('R'=rover for YOLO objects)
    std::string object_class;                       // Object class from YOLO (e.g., "plant", "traffic_light")
    geometry_msgs::msg::TransformStamped tf_object; // 6DOF pose in map frame
    int n_observations;                             // Number of observations for this object
};
```

### Robot Identification System

The system uses semantic robot IDs to track detection sources and fusion status:
- **'D'**: Detection exclusively from drone
- **'R'**: Detection exclusively from rover  
- **'M'**: Multi-robot fusion (marker seen by both drone and rover)

This enables proper attribution and debugging of detection quality.

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

### Multi-Robot Cross-Platform Fusion

The system implements a unified detection vector (`_detections_vector`) that enables true cross-robot fusion:

1. **Unified Data Structure**: Single vector stores detections from all sources
2. **Cross-Platform Matching**: Detections with same `marker_id` are fused regardless of robot source
3. **Semantic Robot Assignment**: Robot ID evolves from 'D'/'R' to 'M' when both robots contribute
4. **Single TF Publication**: One transform per marker with maximum achievable accuracy

### Weighted Average Update

When a marker or object is detected multiple times, the system updates its position using weighted averaging:

```cpp
// Calculate weights based on observation count
int n_obs = detection.n_observations;
double weight_old = (double)n_obs / (n_obs + 1);
double weight_new = 1.0 / (n_obs + 1);

// Update position components
new_x = old_x * weight_old + measured_x * weight_new;
new_y = old_y * weight_old + measured_y * weight_new; 
new_z = old_z * weight_old + measured_z * weight_new;

// Increment observation counter
detection.n_observations++;
```

This approach:
- Gives higher weight to accumulated observations
- Reduces noise and improves accuracy over time  
- Converges to stable position estimates
- Works seamlessly across different robot platforms

### Multi-Robot Benefits

1. **Enhanced Accuracy**: Drone aerial perspective + rover ground-level observations
2. **Redundancy**: System continues if one robot fails
3. **Complementary Viewpoints**: Different angles improve geometric estimation
4. **Unified Navigation**: Single TF tree for both robots to use

### Coordinate Frame Transformations

1. **Camera → Map**: Uses TF buffer to lookup current robot pose
2. **YOLO Processing**: Converts 2D image detections to 3D world coordinates
3. **ArUco Processing**: Direct 6DOF pose from marker detection

## Configuration

### Timer Configuration
The node uses multiple timers for efficient operation:
- **TF Publication Timer**: 100ms (10 Hz) for marker transforms
- **Object TF Timer**: 100ms (10 Hz) for object transforms  
- **Rover TF Listener**: 100ms for coordinate frame updates

### Robot Identification
- **Automatic assignment** based on detection source:
  - **Drone ArUco detections**: `robot = 'D'` (from `/aruco_detector/tf_list`)
  - **Rover ArUco detections**: `robot = 'R'` (from `/rover_aruco_detector/tf_list`)
  - **YOLO object detections**: `robot = 'R'` (from `/yolo/prediction/item_dict`)
- Enables proper multi-robot fusion and debugging

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

## Multi-Robot Deployment

### Single Instance Architecture
The detection publisher should be run as a **single centralized instance** for the entire multi-robot system:

```bash
# Deploy on one robot or dedicated fusion computer
ros2 run detection_publisher_pkg detection_pub_node
```

### Topic Remapping (if needed)
If custom topic names are required:
```bash
ros2 run detection_publisher_pkg detection_pub_node \
  --ros-args \
  -r /aruco_detector/tf_list:=/drone/aruco_detections \
  -r /rover_aruco_detector/tf_list:=/rover/aruco_detections \
  -r /yolo/prediction/item_dict:=/rover/yolo_predictions
```

## Debugging and Monitoring

### Diagnostic Commands
```bash
# View published transforms
ros2 run tf2_tools view_frames.py

# Monitor detection topics
ros2 topic echo /aruco_detector/tf_list
ros2 topic echo /rover_aruco_detector/tf_list
ros2 topic echo /yolo/prediction/item_dict

# Check TF tree
ros2 run tf2_ros tf2_monitor

# View detection statistics
ros2 topic echo /rosout | grep "Multi-robot fusion"
```