
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from zed_msgs.msg import ObjectsStamped
from geometry_msgs.msg import Point, PoseStamped
from rc_interfaces.msg import Cone
import math




class ConeWithDistance:
    """Helper class to store cone with its distance and angle from vehicle"""
    def __init__(self, cone, distance, angle):
        self.cone = cone
        self.distance = distance
        self.angle = angle
class Triangulator(Node):
    def __init__(self):
        super().__init__('triangulator')

        #These are the topics
        self.declare_parameter('cones_topic', '/cone_positions')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('waypoint_topic', '/waypoints')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('lookahead_distance', 10.0)
        self.declare_parameter('num_waypoints', 5)
        self.declare_parameter('min_track_width', 2.0)
        self.declare_parameter('max_track_width', 6.0)


        #topics

        cones_topic = self.get_parameter('cones_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        waypoint_topic = self.get_parameter('waypoint_topic').value
        path_topic = self.get_parameter('path_topic').value
        #min_track_width = self.get_parameter('min_track_width').value
        #max_track_width = self.get_parameter('max_track_width').value

        #self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.min_track_width = float(self.get_parameter('min_track_width').value)
        self.max_track_width = float(self.get_parameter('max_track_width').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.num_waypoints = int(self.get_parameter('num_waypoints').value)



        self.cones_sub = self.create_subscription(ObjectsStamped,cones_topic,self.cones_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,odom_topic,self.odom_callback,10)


        # These are the publishers
        self.waypoint_pub = self.create_publisher(Point, waypoint_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)


        self.latest_odom = None

        self.get_logger().info('Triangulator node')
        self.get_logger().info(f'Lookahead: {self.lookahead_distance}m, Waypoints: {self.num_waypoints}')


    def odom_callback(self, msg):
        """Store the latest odometry data"""
        self.latest_odom = msg

        self.get_logger().debug('Received odometry data')

    def cones_callback(self, msg):
        """Main callback that processes cone data and generates waypoints"""
        self.get_logger().info(f'Received cone data with {len(msg.objects)} objects')

        # check if odometry is available
        if self.latest_odom is None:
            self.get_logger().warn('Odometry not received yet. Skipping processing.')
            return

        # Extract vehicle position
        vehicle_pos = self.latest_odom.pose.pose.position
        vehicle_heading = self.calculate_heading_from_quaternion( self.latest_odom.pose.pose.orientation)


        #first function to extract the cones. this function finds them by their color when called. Its implemented later in the code
        left_cones = self.extract_cones_by_color(msg,  'blue')
        right_cones = self.extract_cones_by_color(msg, 'yellow')


        left_cone = self.find_cone(msg, 'LEFT')
        right_cone = self.find_cone(msg, 'RIGHT')

        self.get_logger().debug(f'found {left_cone} and {right_cone}')
        



    # If we don't have enough cones for pairing, fall back to closest pair if available
        if not left_cones or not right_cones:
            if left_cone and right_cone:
                self.get_logger().info('Using fallback: single closest cone pair')
                waypoint = self.calculate_midpoint(left_cone, right_cone)
                self.waypoint_pub.publish(waypoint)
                self.get_logger().info(f'Published fallback waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})')
            else:
                self.get_logger().warn('No cones available for fallback waypoint')
            return




         # Filter and sort cones by distance and angle
        sorted_left = self.filter_and_sort_cones(left_cones, vehicle_pos, vehicle_heading)
        sorted_right = self.filter_and_sort_cones(right_cones, vehicle_pos, vehicle_heading)




        # Match cone pairs to form track gates. the track gates are just lines between a left cone and a right cone
         #depending on where they are and the orientation of the track whether straight or turning a midpoint between two cones
         #will be calculated.

        cone_pairs = self.match_cone_pairs(sorted_left, sorted_right)

        if not cone_pairs:
            self.get_logger().warn('Could not match any valid cone pairs')
            return

        self.get_logger().info(f'Matched {len(cone_pairs)} cone pairs (gates)')




        # Generate waypoints as midpoints of each gate
        waypoints = []
        for left_cone, right_cone in cone_pairs:
            midpoint = self.calculate_midpoint(left_cone, right_cone)
            waypoints.append(midpoint)



        if not waypoints:
            self.get_logger().warn('No waypoints generated')
            return

        # Publish the first waypoint
        self.waypoint_pub.publish(waypoints[0])
        self.get_logger().info(f'Published waypoint: ({waypoints[0].x:.2f}, {waypoints[0].y:.2f}, {waypoints[0].z:.2f})')

     # Publish the full path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = waypoint
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(waypoints)} waypoints')


    def extract_cones_by_color(self, msg, color):
         """Extract cones of a specific color from the message"""
         cones = []
         for obj in msg.objects:
            if obj.label == color:
                cone = Cone()
                cone.x = obj.position[0]
                cone.y = obj.position[1]
                cone.z = obj.position[2]
                cone.color = 'LEFT' if color == 'blue' else 'RIGHT'


                cones.append(cone)
         return cones

    def find_cone(self, msg, cone_type):
        """Find the closest left or right cone """
        vehicle_pos = self.latest_odom.pose.pose.position
        target_label = 'blue' if cone_type == 'LEFT' else 'yellow'
        min_distance = float('inf')
        result_cone = None

        for obj in msg.objects:
            if obj.label == target_label:
                cone = Cone()
                cone.x = obj.position[0]
                cone.y = obj.position[1]
                cone.z = obj.position[2]
                cone.color = cone_type

                dist = math.sqrt((cone.x - vehicle_pos.x) ** 2 + (cone.y - vehicle_pos.y) ** 2)



                if dist < min_distance:
                    min_distance = dist
                    result_cone = cone

        return result_cone



    def filter_and_sort_cones(self, cones, position, heading):
        """Filter cones by lookahead distance and angle, then sort by distance"""
        cones_with_dist = []

        for cone in cones:
            distance = math.hypot(cone.x - position.x, cone.y - position.y)

            dx = cone.x - position.x
            dy = cone.y - position.y
            cone_angle = math.atan2(dy, dx)
            angle = self.normalize_angle(cone_angle - heading)

            if distance <= self.lookahead_distance and abs(angle) < math.pi / 2.0:
                cones_with_dist.append(ConeWithDistance(cone, distance, angle))

        cones_with_dist.sort(key=lambda c: c.distance)

        return cones_with_dist

    def match_cone_pairs(self, left_cones, right_cones):
        """Match left and right cones to form valid track gates"""
        pairs = []
        left_idx = 0
        right_idx = 0

        while (left_idx < len(left_cones) and
               right_idx < len(right_cones) and
               len(pairs) < self.num_waypoints):

            left = left_cones[left_idx]
            right = right_cones[right_idx]

            if self.is_valid_gate(left.cone, right.cone):
                pairs.append((left.cone, right.cone))
                left_idx += 1
                right_idx += 1
            elif left.distance < right.distance:
                left_idx += 1
            else:
                right_idx += 1

        return pairs


    #The gate is a straight line between a left cone and a right cone.
    def is_valid_gate(self, left_cone, right_cone):
        """Check if two cones form a valid track gate"""
        distance_between_cones = math.sqrt( (left_cone.x - right_cone.x) ** 2 + (left_cone.y - right_cone.y) ** 2 )

        return self.min_track_width <= distance_between_cones <= self.max_track_width

    def calculate_midpoint(self, cone1, cone2):
        """Calculate the midpoint between two cones"""
        midpoint = Point()
        midpoint.x = (cone1.x + cone2.x) / 2.0
        midpoint.y = (cone1.y + cone2.y) / 2.0
        midpoint.z = (cone1.z + cone2.z) / 2.0
        return midpoint
    #This one is used to find the heaading of the car and convert it to a proper angle
    def calculate_heading_from_quaternion(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    #makes sure car isnt outof bounds
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Triangulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
