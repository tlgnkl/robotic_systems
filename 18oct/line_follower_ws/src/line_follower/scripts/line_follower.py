#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from map_loader import MapLoader
from path_planner import PathPlanner


class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        self.rate = rospy.Rate(10)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = None
        self.current_path = None
        self.current_waypoint_index = 0
        self.goal_set = False

        map_loader = MapLoader()
        map_path = rospy.get_param('~map_path', 'maps/line_map.yaml')

        if map_loader.load_map(map_path):
            self.graph = map_loader.get_graph()
            self.map_loader = map_loader
            self.path_planner = PathPlanner(self.graph)
        else:
            rospy.logerr("Не удалось загрузить карту")
            raise rospy.ROSInitException("Map load failed")

        params = rospy.get_param('~control', {})
        self.linear_speed = params.get('linear_speed', 0.1)
        self.angular_speed = params.get('angular_speed', 0.3)
        self.waypoint_tolerance = params.get('waypoint_tolerance', 0.1)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def pixel_to_world(self, pixel_x, pixel_y):
        return self.map_loader.pixel_to_world(pixel_x, pixel_y)

    def set_goal(self, goal_x, goal_y):
        if self.current_pose is None:
            rospy.logwarn_throttle(5.0, "Одометрия еще не получена, ожидание перед установкой цели")
            return False

        pixel_goal_x = int(goal_x / self.map_loader.map_info['resolution'])
        pixel_goal_y = int(goal_y / self.map_loader.map_info['resolution'])

        start_node = self.path_planner.find_nearest_node(
            int(self.current_pose.position.x / self.map_loader.map_info['resolution']),
            int(self.current_pose.position.y / self.map_loader.map_info['resolution'])
        )
        goal_node = self.path_planner.find_nearest_node(pixel_goal_x, pixel_goal_y)

        self.current_path = self.path_planner.dijkstra(start_node, goal_node)
        self.current_waypoint_index = 0

        if self.current_path:
            rospy.loginfo(f"Путь к цели планируется через {len(self.current_path)} точек")
            self.goal_set = True
            return True
        else:
            rospy.logwarn("Не удалось найти путь к цели")
            self.goal_set = False
            return False

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.current_pose and not self.goal_set:
                self.set_goal(2.0, 2.0)

            if self.current_pose and self.current_path:
                self.follow_path()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
