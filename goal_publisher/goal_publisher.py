import rclpy
from rclpy.node import Node
import math
import time

import tf_transformations
from geographiclib.geodesic import Geodesic

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
    # origin_lat =  41.023997
    # origin_long = 28.886453
    # 41.0210371770, 28.8887785020
# 41.021037, 28.888778
# 41.024786, 28.887287
# 41.028010, 28.889435
    goal_lat = 41.021037
    goal_long = 28.888778
    
    geod = Geodesic.WGS84;
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
    hypotenuse = distance = g['s12'] # access distance
    azimuth = g['azi1']
    
    azimuth = math.radians(azimuth)
    q = tf_transformations.quaternion_from_euler(0,0, azimuth)
    x = adjacent = math.cos(azimuth) * hypotenuse
    y = opposite = math.sin(azimuth) * hypotenuse
   
    return x, y, q[2], q[3]

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        timer_period = 500  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.y, msg.pose.position.x, msg.pose.orientation.z, msg.pose.orientation.w  = calc_goal(41.023997, 28.886453, 41.027976, 28.889519)
        
        msg.pose.position.y -= 10
        msg.pose.position.x -= 10
        for i in range(0,20):
            for j in range(0,20):
                msg.pose.position.y += 1
                time.sleep(0.1)
                self.publisher_.publish(msg)
            msg.pose.position.y -= 20
            msg.pose.position.x += 1

        # msg.pose.position.y, msg.pose.position.x, msg.pose.orientation.w, msg.pose.orientation.z  = calc_goal(41.023997, 28.886453, 41.027976, 28.889519)
        
        # msg.pose.position.y -= 5
        # msg.pose.position.x -= 5
        # for i in range(0,10):
        #     for j in range(0,10):
        #         msg.pose.position.y += 1
        #         # time.sleep(1)
        #         self.publisher_.publish(msg)
        #     msg.pose.position.y -= 10
        #     msg.pose.position.x += 1

    def timer_callback(self):
        pass
        # msg = PoseStamped()
        # msg.header.frame_id = 'map'
        # msg.pose.position.y, msg.pose.position.x, msg.pose.orientation.z, msg.pose.orientation.w  = calc_goal(41.023997, 28.886453, 41.027976, 28.889519)
        
        # msg.pose.position.y -= 5
        # msg.pose.position.x -= 5
        # for i in range(0,10):
        #     for j in range(0,10):
        #         msg.pose.position.y += 1
        #         time.sleep(1)
        #         self.publisher_.publish(msg)
        #     msg.pose.position.y -= 10
        #     msg.pose.position.x += 1
       
        # msg.pose.orientation.w = 0.2736437351391402
        # msg.pose.orientation.z = 0.9618311214652497
def main(args=None):
    rclpy.init(args=args)

    goal_pub = GoalPublisher()

    rclpy.spin(goal_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()