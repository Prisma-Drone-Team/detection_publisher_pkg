#include "../include/detection_pub/detection_pub_node.h"

DetectionPublisher::DetectionPublisher()
: rclcpp::Node("detection_publisher") 
{
    _subscription = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/aruco_detector/tf_list", 10,
        std::bind(&DetectionPublisher::detection_cb, this, std::placeholders::_1));

    _subscription_object = this->create_subscription<std_msgs::msg::String>(
        "/yolo/prediction/item_dict", 10,
        std::bind(&DetectionPublisher::object_detection_cb, this, std::placeholders::_1));
    // _subscription_object = this->create_subscription<geometry_msgs::msg::PoseArray>(
    //     "/detected_objects_poses", 10,
    //     std::bind(&DetectionPublisher::object_detection_cb, this, std::placeholders::_1));
    
    _subscription_rover = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/rover_aruco_detector/tf_list", 10,
        std::bind(&DetectionPublisher::detection_rover_cb, this, std::placeholders::_1));

    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    _tf_object_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _timer_tf_out = this->create_wall_timer(
        std::chrono::milliseconds(100),  
        std::bind(&DetectionPublisher::publish_tf, this)
    );
    // _timer_tf_in = this->create_wall_timer(
    //     std::chrono::milliseconds(100),  
    //     std::bind(&DetectionPublisher::listener_tf, this)
    // );
    _timer_object_tf = this->create_wall_timer(
        std::chrono::milliseconds(100),  
        std::bind(&DetectionPublisher::object_tf_pub, this)
    );

    _timer_rover_tf_in = this->create_wall_timer(
        std::chrono::milliseconds(100),  
        std::bind(&DetectionPublisher::listener_rover_tf, this)
    );
    _q_cam_to_map << 1.0, 0.0, 0.0, 0.0;
    _q_cam_to_aruco << 1.0, 0.0, 0.0, 0.0;
    _q_map_to_aruco << 1.0, 0.0, 0.0, 0.0;

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _tf_buffer_rover = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener_rover = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer_rover);

    _ocv_to_pos[0] = tf2::Vector3(0,1,0);
    _ocv_to_pos[1] = tf2::Vector3(0,0,1);
    _ocv_to_pos[2] = tf2::Vector3(1,0,0);
    _ocv_to_pos_tf.setBasis(_ocv_to_pos);
    RCLCPP_INFO(this->get_logger(), "ArUco TF Publisher initialized");
}

void DetectionPublisher::detection_cb( const geometry_msgs::msg::TransformStamped::SharedPtr msg )
{
    // RCLCPP_INFO(this->get_logger(), "Detection received, id: %s", msg->child_frame_id.c_str());
    if( _detections_vector.size() == 0 ) {
        RCLCPP_INFO(this->get_logger(), "First detection, adding to vector");
        Detection new_detection;
        new_detection.robot = 'D';  // Drone detection
        std::cout<<"child frame: "<<msg->child_frame_id<<std::endl;
        std::string id_str = (msg->child_frame_id).c_str(); // Extract substring after "marker_id"
        RCLCPP_INFO(this->get_logger(), "1");
        geometry_msgs::msg::TransformStamped t;
        try {
            t = _tf_buffer->lookupTransform( "map", msg->child_frame_id,  tf2::TimePointZero);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform non disponibile: %s", ex.what());
            return;
        }
        new_detection.marker_id = std::stoi(id_str);
        new_detection.tf_detection.header.frame_id = "map";
        new_detection.tf_detection.child_frame_id = "marker_id" + msg->child_frame_id;
        new_detection.tf_detection.transform.translation = t.transform.translation;;
        
        new_detection.tf_detection.transform.rotation = t.transform.rotation;
        new_detection.n_observations = 1;
        _detections_vector.push_back(new_detection);
    }
    else {
        _already_present = false;
        _existing_id = false;
        for( size_t i=0; i<_detections_vector.size(); i++ ) {
            if( _detections_vector[i].tf_detection.child_frame_id == "marker_id"+msg->child_frame_id ) {
                RCLCPP_INFO(this->get_logger(), "Marker already present, updating with weighted average");
                _already_present = true;
                _detection_pos = i;
                
                // Update detection using weighted average
                geometry_msgs::msg::TransformStamped t;
                try {
                    t = _tf_buffer->lookupTransform( "map", msg->child_frame_id,  tf2::TimePointZero);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Transform not available for update: %s", ex.what());
                    break;
                }
                
                // Calculate weighted average for position fusion
                int n_obs = _detections_vector[i].n_observations;
                double weight_old = (double)n_obs / (n_obs + 1);
                double weight_new = 1.0 / (n_obs + 1);
                
                _detections_vector[i].tf_detection.transform.translation.x = 
                    weight_old * _detections_vector[i].tf_detection.transform.translation.x + 
                    weight_new * t.transform.translation.x;
                _detections_vector[i].tf_detection.transform.translation.y = 
                    weight_old * _detections_vector[i].tf_detection.transform.translation.y + 
                    weight_new * t.transform.translation.y;
                _detections_vector[i].tf_detection.transform.translation.z = 
                    weight_old * _detections_vector[i].tf_detection.transform.translation.z + 
                    weight_new * t.transform.translation.z;
                
                // Update rotation (replace for quaternion stability)
                _detections_vector[i].tf_detection.transform.rotation = t.transform.rotation;
                
                // Mark as multi-robot fusion if different robot contributes
                if (_detections_vector[i].robot != 'D') {
                    _detections_vector[i].robot = 'M';  // 'M' for Multi-robot fusion
                }
                
                // Increment observation counter
                _detections_vector[i].n_observations++;
                
                RCLCPP_INFO(this->get_logger(), "Multi-robot fusion: marker %d updated with %d total observations", 
                           _detections_vector[i].marker_id, _detections_vector[i].n_observations);
                break;
            }
        }
        if( !_already_present ) {
            Detection new_detection;
            new_detection.robot = 'D';  // Drone detection
            std::string id_str = (msg->child_frame_id).c_str(); // Extract substring after "marker_id"
            geometry_msgs::msg::TransformStamped t;
            try {
                t = _tf_buffer->lookupTransform( "map", msg->child_frame_id,  tf2::TimePointZero);
                
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform non disponibile: %s", ex.what());
                return;
            }
            new_detection.marker_id = std::stoi(id_str);
            new_detection.tf_detection.header.frame_id = "map";
            new_detection.tf_detection.child_frame_id = "marker_id"+msg->child_frame_id;
            new_detection.tf_detection.transform.translation = t.transform.translation;;
            
            new_detection.tf_detection.transform.rotation = t.transform.rotation;
            new_detection.n_observations = 1;
            _detections_vector.push_back(new_detection);
            // RCLCPP_INFO(this->get_logger(), "New detection added with id: %d", new_detection.marker_id);
        }
    }

}

void DetectionPublisher::detection_rover_cb( const geometry_msgs::msg::TransformStamped::SharedPtr msg )
{
    // RCLCPP_INFO(this->get_logger(), "Detection received, id: %s", msg->child_frame_id.c_str());
    if( _detections_vector.size() == 0 ) {
        RCLCPP_INFO(this->get_logger(), "First detection, adding to vector");
        Detection new_detection;
        new_detection.robot = 'R';  // Rover detection
        std::string id_str = msg->child_frame_id.c_str(); // Extract substring after "marker_id"
        geometry_msgs::msg::TransformStamped t;
        try {
            t = _tf_buffer_rover->lookupTransform( "map", msg->child_frame_id,  tf2::TimePointZero);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
            return;
        }
        new_detection.marker_id = std::stoi(id_str);
        new_detection.tf_detection.header.frame_id = "map";
        new_detection.tf_detection.child_frame_id = "marker_id"+msg->child_frame_id;
        new_detection.tf_detection.transform.translation = t.transform.translation;        
        new_detection.tf_detection.transform.rotation = t.transform.rotation;
        new_detection.n_observations = 1;
        _detections_vector.push_back(new_detection);
    }
    else {
        _already_present_rover = false;
        for( size_t i=0; i<_detections_vector.size(); i++ ) {
            if( _detections_vector[i].tf_detection.child_frame_id == "marker_id"+msg->child_frame_id ) {
                _already_present_rover = true;
                _detection_pos = i;
                
                // Multi-robot fusion using weighted average
                geometry_msgs::msg::TransformStamped t;
                try {
                    t = _tf_buffer_rover->lookupTransform( "map", msg->child_frame_id,  tf2::TimePointZero);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Transform not available for rover update: %s", ex.what());
                    break;
                }
                
                // Calculate weighted average for multi-robot fusion
                int n_obs = _detections_vector[i].n_observations;
                double weight_old = (double)n_obs / (n_obs + 1);
                double weight_new = 1.0 / (n_obs + 1);
                
                _detections_vector[i].tf_detection.transform.translation.x = 
                    weight_old * _detections_vector[i].tf_detection.transform.translation.x + 
                    weight_new * t.transform.translation.x;
                _detections_vector[i].tf_detection.transform.translation.y = 
                    weight_old * _detections_vector[i].tf_detection.transform.translation.y + 
                    weight_new * t.transform.translation.y;
                _detections_vector[i].tf_detection.transform.translation.z = 
                    weight_old * _detections_vector[i].tf_detection.transform.translation.z + 
                    weight_new * t.transform.translation.z;
                
                // Update rotation (replace for stability)
                _detections_vector[i].tf_detection.transform.rotation = t.transform.rotation;
                
                // Mark as multi-robot fusion if different robot contributes
                if (_detections_vector[i].robot != 'R') {
                    _detections_vector[i].robot = 'M';  // 'M' for Multi-robot fusion
                }
                
                // Increment observation counter
                _detections_vector[i].n_observations++;
                
                RCLCPP_INFO(this->get_logger(), "Multi-robot fusion: marker %d updated with %d total observations", 
                           _detections_vector[i].marker_id, _detections_vector[i].n_observations);
                break;
            }
        }
        if( !_already_present_rover ) {
            Detection new_detection;
            new_detection.robot = 'R';  // Rover detection
            std::string id_str = msg->child_frame_id.c_str(); // Extract substring after "marker_id"
            geometry_msgs::msg::TransformStamped t;
            try {
                t = _tf_buffer_rover->lookupTransform( "map", msg->child_frame_id,  tf2::TimePointZero);
                
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
            }
            new_detection.marker_id = std::stoi(id_str);
            new_detection.tf_detection.header.frame_id = "map";
            new_detection.tf_detection.child_frame_id = "marker_id"+msg->child_frame_id;
            new_detection.tf_detection.transform.translation = t.transform.translation;        
            new_detection.tf_detection.transform.rotation = t.transform.rotation;
            new_detection.n_observations = 1;
            _detections_vector.push_back(new_detection);
            RCLCPP_INFO(this->get_logger(), "New detection added with id: %d", new_detection.marker_id);
        }
    }

}

void DetectionPublisher::object_detection_cb( const std_msgs::msg::String::SharedPtr msg )
{
    try {
        // Estrai il JSON dalla stringa (rimuovi "data: '")
        std::string json_str = extract_json_from_string(msg->data);
        
        // Parsing del JSON
        json json_data = json::parse(json_str);
        
        // Estrai i campi
        parse_json_data(json_data);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Errore nel parsing JSON: %s", e.what());
    }
}

// void DetectionPublisher::object_detection_cb( const geometry_msgs::msg::PoseArray msg ) {

//         if( _object_vector.size() == 0 ) {
//             ObjectDetection new_object;
//             new_object.robot = _robot_id;
//             new_object.object_class = object_class;
//             new_object.tf_object.header.frame_id = "rover/camera_rgb_optical_frame";
//             _p_temp_c_rover << x,
//             y,
//             0.0,
//             1.0;
//             _q_cam_to_aruco_rover << 1.0,
//                               0.0,
//                               0.0,
//                               0.0;
//             _R_cam_to_aruco_rover = QuatToMat( _q_cam_to_aruco_rover );
//             _T_cam_to_aruco_rover.block<3,3>(0,0) = _R_cam_to_aruco_rover;
//             _T_cam_to_aruco_rover.block<3,1>(0,3) = _p_temp_c_rover.head<3>();
//             _T_cam_to_aruco_rover(3,3) = 1.0;
//             // Transform from camera to map frame: _T_cam_to
//             // _T_cam_to_aruco 
//             _T_map_to_aruco_rover =  _T_cam_to_map_rover* _T_cam_to_aruco_rover;
//             _q_map_to_aruco_rover = r2quat( _T_map_to_aruco_rover.block<3,3>(0,0) );
//             // new_object.tf_object.header.frame_id = "rover/map";
//             new_object.tf_object.child_frame_id = "object_"+object_class;
//             new_object.tf_object.transform.translation.x = 0.0;
//             new_object.tf_object.transform.translation.y = 0.0;
//             new_object.tf_object.transform.translation.z = 0.0;
//             new_object.tf_object.transform.rotation.x = 0.0;
//             new_object.tf_object.transform.rotation.y = 0.0;
//             new_object.tf_object.transform.rotation.z = 0.0;
//             new_object.tf_object.transform.rotation.w = 1.0;
//             _objects_vector.push_back(new_object);
//         }

// }

std::string DetectionPublisher::extract_json_from_string(const std::string& input)
{
    // Cerca l'inizio del JSON dopo "data: '"
    size_t start_pos = input.find("data: '");
    if (start_pos == std::string::npos) {
        // Se non trova "data: '", prova a parsare direttamente
        return input;
    }
    
    // Estrai dalla fine di "data: '" fino all'ultimo apostrofo
    start_pos += 7; // lunghezza di "data: '"
    size_t end_pos = input.find_last_of("'");
    
    if (end_pos == std::string::npos || end_pos <= start_pos) {
        throw std::runtime_error("Formato stringa non valido");
    }
    
    return input.substr(start_pos, end_pos - start_pos);
}

void DetectionPublisher::parse_json_data(const json& json_data)
{
    std::string object_class;
    double x=0.0, y=0.0, z=0.0, w=0.0;
    
    // Accedi ai dati JSON
    if (json_data.contains("item_0")) {
        auto item_0 = json_data["item_0"];
        
        // Estrai la classe
        if (item_0.contains("class")) {
            object_class = item_0["class"];
            for (char &c : object_class) {
                if(c==' ') {
                    c='_';
                }
            }

            //convert from coco to leonardo
            if(object_class == "potted_plant"){
                object_class = "plant";
            }
            //else if(object_class == "teddy_bear"){
            //    //do nothing
            //}
            //else if(object_class == "fire_hydrant"){
            //    //do nothing
            //}
            //else if(object_class == "traffic_light"){
            //    //do nothing
            //}
            // RCLCPP_INFO(this->get_logger(), "Classe: %s", object_class.c_str());
        }
        // Estrai la posizione (array)
        if (item_0.contains("position")) {
            auto position_array = item_0["position"];
            
            if (position_array.is_array() && position_array.size() >= 3) {
                x = position_array[0];
                y = position_array[1];
                z = position_array[2];
                w = position_array[3];
                if(std::abs(z)>4.0){
                    return;
                }
                
                // RCLCPP_INFO(this->get_logger(), 
                //            "Posizione: x=%.3f, y=%.3f, z=%.3f, d=%.3f", 
                //            x, y, z, w);
                
            }
        }
    }
    // std::cout<<_objects_vector.size()<<std::endl;
    if( _objects_vector.size() == 0 ) {
        ObjectDetection new_object;
        new_object.robot = 'R';  // YOLO objects from rover
        new_object.object_class = object_class;
        new_object.tf_object.header.frame_id = "rover/camera_rgb_optical_frame";
        _p_temp_c_rover << x,
        0.0,
        z,
        1.0;
        _q_cam_to_aruco_rover << 1.0,
                          0.0,
                          0.0,
                          0.0;
        _R_cam_to_aruco_rover = QuatToMat( _q_cam_to_aruco_rover );
        _T_cam_to_aruco_rover.block<3,3>(0,0) = _R_cam_to_aruco_rover;
        _T_cam_to_aruco_rover.block<3,1>(0,3) = _p_temp_c_rover.head<3>();
        _T_cam_to_aruco_rover(3,3) = 1.0;

        _T_map_to_aruco_rover =  _T_cam_to_map_rover*_T_cam_to_aruco_rover;
        _q_map_to_aruco_rover = r2quat( _T_map_to_aruco_rover.block<3,3>(0,0) );
        tf2::Transform T_c2a_rover, T_final;
        tf2::Stamped<tf2::Transform> T_m2c;
        tf2::Matrix3x3 R;
        R.setIdentity();
        tf2::Vector3 p_c2a;
        p_c2a.setValue(x,0.0,z);
        T_c2a_rover.setOrigin(p_c2a);
        T_c2a_rover.setBasis(R);
        tf2::fromMsg(_t_rover, T_m2c);
        T_final.mult(T_m2c, T_c2a_rover);


        new_object.tf_object.header.frame_id = "map";
        new_object.tf_object.child_frame_id = object_class;
        new_object.tf_object.transform = tf2::toMsg(T_final);
        new_object.n_observations = 1;  // Initialize observation counter
        _objects_vector.push_back(new_object);
    } 
    else {
        _object_already_present = false;
        int object_pos = -1;
        for( size_t i=0; i<_objects_vector.size(); i++ ) {
            if( _objects_vector[i].object_class == object_class ) {
                _object_already_present = true;
                object_pos = i;
                
                // Aggiorna l'oggetto esistente con media pesata
                tf2::Transform T_c2a_rover, T_final;
                tf2::Stamped<tf2::Transform> T_m2c;
                tf2::Matrix3x3 R;
                R.setIdentity();
                tf2::Vector3 p_c2a;
                p_c2a.setValue(x, 0.0, z);
                T_c2a_rover.setOrigin(p_c2a);
                T_c2a_rover.setBasis(R);
                tf2::fromMsg(_t_rover, T_m2c);
                T_final.mult(T_m2c, T_c2a_rover);
                
                // Calcola media pesata delle posizioni
                int n_obs = _objects_vector[i].n_observations;
                double weight_old = (double)n_obs / (n_obs + 1);
                double weight_new = 1.0 / (n_obs + 1);
                
                tf2::Vector3 old_pos = tf2::Vector3(
                    _objects_vector[i].tf_object.transform.translation.x,
                    _objects_vector[i].tf_object.transform.translation.y,
                    _objects_vector[i].tf_object.transform.translation.z
                );
                tf2::Vector3 new_pos = T_final.getOrigin();
                tf2::Vector3 avg_pos = old_pos * weight_old + new_pos * weight_new;
                
                _objects_vector[i].tf_object.transform.translation.x = avg_pos.x();
                _objects_vector[i].tf_object.transform.translation.y = avg_pos.y();
                _objects_vector[i].tf_object.transform.translation.z = avg_pos.z();
                
                // Per la rotazione, mantieni quella attuale (identità per oggetti YOLO)
                _objects_vector[i].tf_object.transform.rotation = tf2::toMsg(T_final.getRotation());
                
                // Increment observation counter
                _objects_vector[i].n_observations++;
                
                RCLCPP_INFO(this->get_logger(), "Object %s updated: %d total observations", 
                           object_class.c_str(), _objects_vector[i].n_observations);
                break;
            }
        }
        if( !_object_already_present ) {
            RCLCPP_INFO(this->get_logger(), "----NEW-----New object added");
        ObjectDetection new_object;
        new_object.robot = 'R';  // YOLO objects from rover
        new_object.object_class = object_class;
        new_object.tf_object.header.frame_id = "rover/camera_rgb_optical_frame";
        _p_temp_c_rover << x,
        0.0,
        z,
        1.0;
        _q_cam_to_aruco_rover << 1.0,
                          0.0,
                          0.0,
                          0.0;
        _R_cam_to_aruco_rover = QuatToMat( _q_cam_to_aruco_rover );
        _T_cam_to_aruco_rover.block<3,3>(0,0) = _R_cam_to_aruco_rover;
        _T_cam_to_aruco_rover.block<3,1>(0,3) = _p_temp_c_rover.head<3>();
        _T_cam_to_aruco_rover(3,3) = 1.0;

        _T_map_to_aruco_rover =  _T_cam_to_map_rover*_T_cam_to_aruco_rover;
        _q_map_to_aruco_rover = r2quat( _T_map_to_aruco_rover.block<3,3>(0,0) );

        tf2::Transform T_c2a_rover, T_final;
        tf2::Stamped<tf2::Transform> T_m2c;
        tf2::Matrix3x3 R;
        R.setIdentity();
        tf2::Vector3 p_c2a;
        p_c2a.setValue(x,0.0,z);
        T_c2a_rover.setOrigin(p_c2a);
        T_c2a_rover.setBasis(R);
        tf2::fromMsg(_t_rover, T_m2c);
        T_final.mult(T_m2c, T_c2a_rover);


        new_object.tf_object.header.frame_id = "map";
        new_object.tf_object.child_frame_id = object_class;
        new_object.tf_object.transform = tf2::toMsg(T_final);
        new_object.n_observations = 1;  // Initialize observation counter
        _objects_vector.push_back(new_object);
        }
    }
    // Se ci sono più items, puoi iterarli
    for (auto& [key, value] : json_data.items()) {
        if (key.find("item_") == 0) { // Tutti gli items che iniziano con "item_"
            RCLCPP_DEBUG(this->get_logger(), "Processando %s", key.c_str());
        }
    }
}

void DetectionPublisher::listener_tf() {
    geometry_msgs::msg::TransformStamped t;
    try {
        t = _tf_buffer->lookupTransform( "map", "zed_front_left_camera_optical_frame",  tf2::TimePointZero);
        
        // RCLCPP_INFO(this->get_logger(), 
        //            "Transform found: [%f, %f, %f]", 
        //            t.transform.translation.x,
        //            t.transform.translation.y,
        //            t.transform.translation.z);
    } catch (tf2::TransformException &ex) {
        // RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
    }

    _p_cam_to_map << t.transform.translation.x,
                     t.transform.translation.y,
                     t.transform.translation.z;
    _q_cam_to_map << t.transform.rotation.w,
                     t.transform.rotation.x,
                     t.transform.rotation.y,
                     t.transform.rotation.z;
    _R_cam_to_map = QuatToMat( _q_cam_to_map );
    _T_cam_to_map.block<3,3>(0,0) = _R_cam_to_map;
    _T_cam_to_map.block<3,1>(0,3) = _p_cam_to_map;
    _T_cam_to_map(3,3) = 1.0;
    
}

void DetectionPublisher::listener_rover_tf() {
    geometry_msgs::msg::TransformStamped t;
    try {
        _t_rover = _tf_buffer_rover->lookupTransform( "map","rover/camera_rgb_optical_frame",    tf2::TimePointZero);
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform non disponibile: %s", ex.what());
        return;
    }

    _p_cam_to_map_rover << t.transform.translation.x,
                     t.transform.translation.y,
                     t.transform.translation.z;
    _q_cam_to_map_rover << t.transform.rotation.w,
                     t.transform.rotation.x,
                     t.transform.rotation.y,
                     t.transform.rotation.z;
    _R_cam_to_map_rover = QuatToMat( _q_cam_to_map_rover );
    _T_cam_to_map_rover.block<3,3>(0,0) = _R_cam_to_map_rover;
    _T_cam_to_map_rover.block<3,1>(0,3) = _p_cam_to_map_rover;
    _T_cam_to_map_rover(3,3) = 1.0;
    
}


void DetectionPublisher::publish_tf() {
    // Publisher TF for unified detections from all robots
    for( size_t i=0; i<_detections_vector.size(); i++ ) {
        rclcpp::Clock clock;

        // Publish main marker transform
        geometry_msgs::msg::TransformStamped tf_to_publish;
        tf_to_publish = _detections_vector[i].tf_detection;
        tf_to_publish.header.stamp = clock.now();
        _tf_broadcaster->sendTransform(tf_to_publish);

        // Publish rotation correction transform
        geometry_msgs::msg::TransformStamped tf_to_ocv;
        tf_to_ocv.header.frame_id = _detections_vector[i].tf_detection.child_frame_id;
        tf_to_ocv.header.stamp = clock.now();
        tf_to_ocv.child_frame_id = _detections_vector[i].tf_detection.child_frame_id+"_rot";
        tf_to_ocv.transform.rotation = tf2::toMsg(_ocv_to_pos_tf.getRotation());
        _tf_broadcaster->sendTransform(tf_to_ocv);

        // Publish target approach transform
        geometry_msgs::msg::TransformStamped tf_target_publish;
        tf_target_publish.header.stamp = clock.now();
        // std::cout<<"Time: "<<tf_target_publish.header.stamp.sec<<"."<<tf_target_publish.header.stamp.nanosec;
        tf_target_publish.header.frame_id = _detections_vector[i].tf_detection.child_frame_id+"_rot";
        tf_target_publish.child_frame_id = _detections_vector[i].tf_detection.child_frame_id+".target";
        tf_target_publish.transform.translation.x = 1.0; 
        tf_target_publish.transform.translation.y = 0.0; 
        tf_target_publish.transform.translation.z = 0.0; 
        tf2::Quaternion q_rot_z;
        q_rot_z.setRPY(0, 0, M_PI);
        tf_target_publish.transform.rotation = tf2::toMsg(q_rot_z);
        // tf_target_publish.transform.rotation.x = 0.0;
        // tf_target_publish.transform.rotation.y = 0.0;
        // tf_target_publish.transform.rotation.z = 0.0;
        // tf_target_publish.transform.rotation.w = 1.0;

        // _tf_target_broadcaster->sendTransform(tf_target_publish);
        _tf_broadcaster->sendTransform(tf_target_publish);
    }   
}

void DetectionPublisher::object_tf_pub() {
    // std::cout<<"\n Publisher TF for ID: [ ";
    for( size_t i=0; i<_objects_vector.size(); i++ ) {
        
        rclcpp::Clock clock;
        geometry_msgs::msg::TransformStamped tf_to_publish;
        // tf_to_publish.header.stamp = this->get_clock()->now();
        
        tf_to_publish = _objects_vector[i].tf_object;
        tf_to_publish.header.stamp = clock.now();
        _tf_broadcaster->sendTransform(tf_to_publish);
        // std::cout<<_objects_vector[i].object_class<<", ";

        geometry_msgs::msg::TransformStamped tf_target_publish;

        tf_target_publish.header.stamp = clock.now();
        // std::cout<<"Time: "<<tf_target_publish.header.stamp.sec<<"."<<tf_target_publish.header.stamp.nanosec;
        tf_target_publish.header.frame_id = _objects_vector[i].tf_object.child_frame_id;
        tf_target_publish.child_frame_id = _objects_vector[i].tf_object.child_frame_id+".target";
        tf_target_publish.transform.translation.x = 0.0; 
        tf_target_publish.transform.translation.y = -1.65; 
        tf_target_publish.transform.translation.z = 0.0; 
        tf_target_publish.transform.rotation.x = 0.0;
        tf_target_publish.transform.rotation.y = 0.0;
        tf_target_publish.transform.rotation.z = 0.0;
        tf_target_publish.transform.rotation.w = 1.0;
        std::cout<<"Publishing here\n";
        // _tf_target_broadcaster->sendTransform(tf_target_publish);
        _tf_object_broadcaster->sendTransform(tf_target_publish);
    }   
}


int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the DetectionPublisher node
    auto node = std::make_shared<DetectionPublisher>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}