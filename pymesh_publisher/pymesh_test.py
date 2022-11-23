import os, sys
import json
import numpy as np
import math
import rclpy
import cv2
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
import tf2_ros as tf2
from geometry_msgs.msg import Quaternion

from glob import glob

class PyMeshPublisher(Node):
    def __init__(self):
        super().__init__('pymesh_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'pymesh', 10)
        self.img_publisher = self.create_publisher(Image, 'pymesh_img', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.img_timer = self.create_timer(timer_period, self.publish_img_callback)

        self.all_json_paths = sorted(glob("/home/za/Documents/BEVFormer/tmp/*.json"))
        self.all_img_paths = sorted(glob("/home/za/Documents/BEVFormer/tmp/*.jpg"))
        self.index = 0
        self.detections = None
        self.images = None
        self.CLASS_NAMES = ['car', 'truck', 'construction_vehicle', 'bus', 'trailer', 'barrier', 'motorcycle', 'bicycle', 'pedestrian', 'traffic_cone']

    def publish_callback(self):
        self.read_from_json()
        marker_array = MarkerArray()
        bboxes = np.array(self.detections["bboxes"])
        scores = np.array(self.detections["scores"])
        labels = np.array(self.detections["labels"])
        assert len(bboxes) == len(scores) == len(labels)
        for i in range(len(bboxes)):
            bbox = bboxes[i]
            score = scores[i]
            label = labels[i]
            class_name = self.CLASS_NAMES[label]
            marker = self.make_mesh_marker(i, bbox[0], bbox[1], bbox[2], bbox[6], class_name)
            if marker is not None:
                # cube = self.make_cube_marker(i+10000, bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5], bbox[6], class_name)
                # marker_array.markers.append(cube)
                marker_array.markers.append(marker)
        self.publisher_.publish(marker_array)
        self.get_logger().info('Publishing num of markers: "%d"' % len(marker_array.markers))
    
    def publish_img_callback(self):
        img_msg = Image()
        img_msg.header.frame_id = "map"
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.height = self.images.shape[0]
        img_msg.width = self.images.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = False
        img_msg.step = self.images.shape[1] * 3
        img_msg.data = self.images.tobytes()
        self.img_publisher.publish(img_msg)
        self.get_logger().info('Publishing image: "%s"' % self.all_img_paths[self.index])
    
    def read_from_json(self):
        json_path = self.all_json_paths[self.index]
        img_path = self.all_img_paths[self.index]
        self.index += 1
        if self.index >= len(self.all_json_paths) - 1:
            self.index = 0
        self.detections = json.load(open(json_path, "r"))
        self.images = cv2.imread(img_path) 
        self.images = cv2.resize(self.images, None, fx=0.2, fy=0.2)
    
    def choose_class_model(self, class_name):
        model_path = None
        scale = None
        angle_offset = None
        yaw_offset = 0
        if class_name == 'car':
            angle_offset = 0.0
            scale = 0.01
            yaw_offset = 0
            # yaw_offset = np.pi
            model_path = "file:///home/za/dev_ws/src/pymesh_publisher/models/car/Hybrid.obj"
        elif class_name == "truck":
            angle_offset = 0.0
            scale = 1.0 
            yaw_offset = np.pi
            # yaw_offset = 0
            model_path = "file:///home/za/dev_ws/src/pymesh_publisher/models/truck/pickup.dae"
        elif class_name == "bus":
            angle_offset = 0.0
            scale = 0.008
            yaw_offset = 0
            # yaw_offset = np.pi
            model_path = "file:///home/za/dev_ws/src/pymesh_publisher/models/bus/bus.obj"
        elif class_name == "bicycle" or class_name == "motorcycle":
            angle_offset = np.pi/2.0
            scale = 1.5
            yaw_offset = 0
            # yaw_offset = np.pi
            model_path = "file:///home/za/dev_ws/src/pymesh_publisher/models/bike/bicycle.obj"
        elif class_name == "pedestrian":
            angle_offset = 0.0
            scale = 1.0 
            yaw_offset = 0
            # yaw_offset = np.pi
            model_path = "file:///home/za/dev_ws/src/pymesh_publisher/models/people/casual_female.dae"
        else:
            scale = None
            angle_offset = None
            model_path = None
            yaw_offset = 0
        return model_path, scale, angle_offset, yaw_offset
    
    def make_mesh_marker(self, id, x, y, z, heading_angle, class_name):
        model_path, scale, angle_offset, yaw_offset = self.choose_class_model(class_name)
        if model_path is None:
            return None
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pymesh" 
        marker.id = id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z 

        angle = heading_angle
        orientation = self.quaternion_from_euler(angle_offset, 0.0, -angle + yaw_offset)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.mesh_resource = model_path
        # marker.mesh_use_embedded_materials = True
        marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return marker
    
    def make_cube_marker(self, id, x, y, z, w, h, d, heading_angle, class_name):
        model_path, scale, angle_offset, yaw_offset = self.choose_class_model(class_name)
        if model_path is None:
            return None
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pymesh" 
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z 

        angle = heading_angle
        orientation = self.quaternion_from_euler(angle_offset, 0.0, -angle + yaw_offset)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.scale.x = w
        marker.scale.y = h
        marker.scale.z = d

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return marker
    
    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q



def main(args=None):
    rclpy.init(args=args)
    pymesh_publisher = PyMeshPublisher()
    rclpy.spin(pymesh_publisher)
    pymesh_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()