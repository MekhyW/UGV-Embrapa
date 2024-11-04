import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path 
from tf2_ros import Buffer, TransformListener
from geodesy.utm import fromLatLong
import tf2_geometry_msgs
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient

class GpsInterface(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__("gps_interface")
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.waypoint_sub = self.create_subscription(
            Path, "/gps_goal", self.waypoint_cb, 1)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def waypoint_cb(self, msg: Path):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """ 
        wps = []

        for p in msg.poses:
            utm_point = fromLatLong(p.pose.position.x, p.pose.position.y)

            if p.pose.position.x < 0:
                utm_point.northing += 10000000.0

            pose = PoseStamped()
            pose.pose.position.x = utm_point.easting
            pose.pose.position.y = utm_point.northing
        
            gps_to_map = self.tf_buffer.lookup_transform('map', 'utm', rclpy.time.Time())
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose, gps_to_map)
            print(pose_map)

            pose_map.pose.position.z = 0.0
            pose_map.pose.orientation.w = 1.0
            pose_map.pose.orientation.x = 0.0
            pose_map.pose.orientation.y = 0.0
            pose_map.pose.orientation.z = 0.0
            
            wps.append(pose_map)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = wps
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()