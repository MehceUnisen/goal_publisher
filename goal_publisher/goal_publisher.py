import rclpy
from rclpy.node import Node
import math
import time

import tf_transformations
from geographiclib.geodesic import Geodesic
from autoware_adapi_v1_msgs.msg import RouteState
from geometry_msgs.msg import PoseStamped

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
    # 41.0210371770, 28.8887785020
# 41.021037, 28.888778
# 41.024786, 28.887287
# 41.028010, 28.889435
# 41.025194, 28.887101
    # goal_lat = 41.021037
    # goal_long = 28.888778
    
    geod = Geodesic.WGS84;
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
    hypotenuse = distance = g['s12'] # access distance
    azimuth = g['azi1']
    azimuth = math.radians(azimuth)
    q = tf_transformations.quaternion_from_euler(0,0, azimuth)
    x = math.cos(azimuth) * hypotenuse
    y = math.sin(azimuth) * hypotenuse
   
    return x, y, q[2], q[3]

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.subscriber_ = self.create_subscription(RouteState, '/planning/mission_planning/route_state', self.route_callback, 10)
        timer_period = 500  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.y, msg.pose.position.x, msg.pose.orientation.z, msg.pose.orientation.w  = calc_goal(41.023997, 28.886453,41.021037, 28.888778)
        
        msg.pose.position.y -= 5
        msg.pose.position.x -= 5
        for i in range(0,10):
            for j in range(0,10):
                self.publisher_.publish(msg)
                
                time.sleep(0.1)
                # rest of the indented block coveres the case that
                # lane orientation mismatch probability
                msg.pose.orientation.z = -msg.pose.orientation.z
                self.publisher_.publish(msg)
                msg.pose.orientation.z = -msg.pose.orientation.z
                time.sleep(0.1)
                msg.pose.orientation.w , msg.pose.orientation.z = msg.pose.orientation.z , msg.pose.orientation.w 
                self.publisher_.publish(msg)
                time.sleep(0.1)
                msg.pose.position.y += 1

            msg.pose.position.y -= 10


            msg.pose.position.x += 1

    def __del__(self):
       pass 

    def route_callback(self, msg):
        if msg.state == 2:
            del self

    def timer_callback(self):
        pass
        
def main(args=None):
    rclpy.init(args=args)

    goal_pub = GoalPublisher()

    rclpy.spin(goal_pub)
    goal_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()