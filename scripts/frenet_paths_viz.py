#!/usr/bin/python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from jackal_info_gain.msg import FrenetPaths

class FrenetViz(object):
    def __init__(self, frenet_path_topic):
        self._frenet_paths_sub = rospy.Subscriber("/" + frenet_path_topic + "_viz", FrenetPaths, self._frenet_paths_cb, queue_size=1)
        self._paths = None
        self._marker_pub = rospy.Publisher(frenet_path_topic + "_viz_marker", MarkerArray, latch=True, queue_size=10)

        self.marker_array = MarkerArray()

    def _generate_marker_msg(self):
        """_summary_
        """
        
        marker_array = MarkerArray()
        
        num_paths = len(self._paths.paths)
        for _i in range(num_paths):
            line_marker = FrenetViz._generate_marker_from_path(self._paths[_i])
            marker_array.markers.append(line_marker)
        return marker_array
    
    @staticmethod
    def _generate_marker_from_path(path):
        """generate visual marker for frenet path

        Args:
            path (_type_): frenet path
        """
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        
        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        
        # marker color 
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.points = []
        
        for _i in range(len(path.poses)):
            p = Point()
            p.x = path.poses[_i].pose.position.x
            p.y = path.poses[_i].pose.position.y
            p.z = path.poses[_i].pose.position.z
            
            marker.points.append(p)
            
        return marker
        
    def _frenet_paths_cb(self, msg):

        print("New message arrived")
        self._paths = msg
        
        marker_msg = self._generate_marker_msg()
        
        self._marker_pub.publish(marker_msg)
        print("published new msg")
        
if __name__ == '__main__':

    rospy.init_node('markers_simulate')

    topic = rospy.get_param("/jig/frenet_paths_pub_topic")
    print("Topic to subscribe: ", topic)
    frenet_viz = FrenetViz(frenet_path_topic=topic)

    rospy.spin()