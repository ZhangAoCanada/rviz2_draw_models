#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <jsoncpp/json/json.h>

class RvizDrawCube : public rclcpp::Node
{
public:
    RvizDrawCube()
        : Node("mesh_test")
    {
        sfusion_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "mesh_visualization", 10);

        auto draw_frequency = std::chrono::milliseconds(10);
        sfusion_timer_ = this->create_wall_timer(draw_frequency, std::bind(&RvizDrawCube::sfusion_publishDrawing_callback, this));
    }

private:

    void sfusion_publishDrawing_callback()
    {
        visualization_msgs::msg::MarkerArray markerArray;

        visualization_msgs::msg::Marker marker1;
        makeMeshMarker(marker1, 0, 0, 0, 0, 0.005, 0.0, 
            "file:///home/za/dev_ws/src/mesh_publisher/models/car/Hybrid.obj");
        markerArray.markers.push_back(marker1);

        visualization_msgs::msg::Marker marker2;
        makeMeshMarker(marker2, 1, 5, 0, 0, 0.5, 0.0, 
            "file:///home/za/dev_ws/src/mesh_publisher/models/truck/pickup.dae");
        markerArray.markers.push_back(marker2);

        visualization_msgs::msg::Marker marker3;
        makeMeshMarker(marker3, 2, -5, 0, 0, 0.005, 0.0,  
            "file:///home/za/dev_ws/src/mesh_publisher/models/bus/bus.obj");
        markerArray.markers.push_back(marker3);

        visualization_msgs::msg::Marker marker4;
        makeMeshMarker(marker4, 3, 10, 0, 0, 0.8, M_PI/2,  
            "file:///home/za/dev_ws/src/mesh_publisher/models/bike/bicycle.obj");
        markerArray.markers.push_back(marker4);

        visualization_msgs::msg::Marker marker5;
        makeMeshMarker(marker5, 4, -10, 0, 0, 0.5, 0.0, 
            "file:///home/za/dev_ws/src/mesh_publisher/models/people/casual_female.dae");
        markerArray.markers.push_back(marker5);
       
        sfusion_publisher_->publish(markerArray);
        RCLCPP_INFO(this->get_logger(), "Publishing num of markers: %d", static_cast<int>(markerArray.markers.size()));
    }

    void makeMeshMarker(visualization_msgs::msg::Marker &marker, const int id,  const float x, const float y, const float z, const float scale, const float angleOffset, const std::string& mesh_path)
    {
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "mesh";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        tf2::Quaternion orientation;
        // float vx = static_cast<float>(object_msg.long_vel) / 1000.0f;
        // float vy = static_cast<float>(object_msg.lat_vel) / 1000.0f;
        // float angle = atan2(vy, vx);
        float angle = static_cast<float>(0) / 1000.0f * M_PI / 180.0f;
        orientation.setRPY(angleOffset, 0.0, angle);
        marker.pose.orientation.x = orientation.x();
        marker.pose.orientation.y = orientation.y();
        marker.pose.orientation.z = orientation.z();
        marker.pose.orientation.w = orientation.w();

        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "file:///home/za/dev_ws/src/mesh_publisher/models/suv/suv.obj";
        marker.mesh_resource = mesh_path;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1); 
    }

    void readDetections(const std::string& json_path)
    {
        Json::Reader reader;
        std::ifstream ifs(json_path);
        if (!reader.parse(ifs, json_val_, false))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse json file: %s", json_path.c_str());
            return;
        }
    }

    void chooseClassModel(const std::string& class_name)
    {
        if (class_name == "car")
        {
            model_path_ = "file:///home/za/dev_ws/src/mesh_publisher/models/car/Hybrid.obj";
        }
        else if (class_name == "truck")
        {
            model_path_ = "file:///home/za/dev_ws/src/mesh_publisher/models/truck/pickup.dae";
        }
        else if (class_name == "bus")
        {
            model_path_ = "file:///home/za/dev_ws/src/mesh_publisher/models/bus/bus.obj";
        }
        else if (class_name == "bike")
        {
            model_path_ = "file:///home/za/dev_ws/src/mesh_publisher/models/bike/bicycle.obj";
        }
        else if (class_name == "person")
        {
            model_path_ = "file:///home/za/dev_ws/src/mesh_publisher/models/people/casual_female.dae";
        }
        else
        {
            std::cerr << "********** Class name not supported" << std::endl;
        }
    }


    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sfusion_publisher_;

    rclcpp::TimerBase::SharedPtr sfusion_timer_;
    visualization_msgs::msg::MarkerArray sfusion_markerArray_;

    std::string model_path_;
    Json::Value json_val_;
};


/*
 * @brief Entry point of visualization.
*/
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizDrawCube>());
    rclcpp::shutdown();
}
