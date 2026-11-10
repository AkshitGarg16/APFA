import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from data_type.msg import Force  
import math

class ControllerNode(Node):


    def __init__(self, goal_global):

        super().__init__('controller_node')
  
        self.goal_lat = goal_global[0]
        self.goal_lon = goal_global[1]

        self.home_lat = None
        self.home_lon = None

        self.compass_hdg = None        
        
        self.ka = 0.1
        self.kr = 400.0
        self.ktheta = 0.02

        self.eps_orient = np.deg2rad(40)
        self.eps_control = 1.0
        self.range_min = 1.0      
        self.range_max = 3.0      
        self.gstar = 2.8 

        self.min_cluster_size = 25

        self.bridge = CvBridge()
        self.ControlVel = Twist()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.DepthcamCallback,
            qos
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.GpsCallback,
            qos
        )

        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.CompassCallback,
            qos
        )

        self.VelocityPublisher = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.cluster_publisher = self.create_publisher(
            Float64,
            '/apfa/num_clusters',
            10
        )
        self.force_publisher = self.create_publisher(
            Force,
            '/apfa/force',
            10
        )
        
        self.OdometryMessage = None
        self.DepthcamMessage = None

        self.fov_horizontal_deg = 87.0
        self.fov_horizontal_rad = np.deg2rad(self.fov_horizontal_deg)

        self.controller_timer = self.create_timer(0.2, self.Controller)

        self.kp_thetavel = 0.8  
        self.kd_thetavel = 0.3  
        self.prev_error_yaw = 0.0
        self.dt = 0.05

    
    def DepthcamCallback(self, msg):
        self.DepthcamMessage = msg

    def GpsCallback(self, msg):
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.get_logger().info(
                f"Home set: lat={self.home_lat}, lon={self.home_lon}"
            )
        self.gps_msg = msg

    def CompassCallback(self, msg):
        self.compass_hdg = msg.data  # degrees (0 = North)

    def global_to_local_xy(self, lat1, lon1, lat2, lon2):
        R = 6378137.0  # meters

        lat1, lon1, lat2, lon2 = map(np.deg2rad, [lat1, lon1, lat2, lon2])

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # --- Haversine distance ---
        a = np.sin(dlat / 2.0)**2 + \
            np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2

        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        distance = R * c

        # --- Initial bearing ---
        bearing = np.arctan2(
            np.sin(dlon) * np.cos(lat2),
            np.cos(lat1) * np.sin(lat2) -
            np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
        )
        print(bearing)
        # --- Convert to local ENU ---
        x = distance * np.cos(bearing)   # East
        y = distance * np.sin(bearing)   # North

        return x, y

    
    # def global_to_local_xy(self, lat1, lon1, lat2, lon2):
    
    # # distance between latitudes
    # # and longitudes
    #     dLat = (lat2 - lat1) * math.pi / 180.0
    #     dLon = (lon2 - lon1) * math.pi / 180.0

    #     # convert to radians
    #     lat1 = (lat1) * math.pi / 180.0
    #     lat2 = (lat2) * math.pi / 180.0

    #     # apply formulae
    #     a = (pow(math.sin(dLat / 2), 2) + 
    #         pow(math.sin(dLon / 2), 2) * 
    #             math.cos(lat1) * math.cos(lat2))
    #     rad = 6371
    #     c = 2 * math.asin(math.sqrt(a))
    #     Bearing = math.atan2(math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1), math.sin(lon2-lon1)*math.cos(lat2))
    #     x = rad * c * math.sin(Bearing)
    #     y = rad * c * math.cos(Bearing)
        
        
        #return x * 1000, y * 1000  # in meters  
        

    def cluster_centroid(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        return np.mean(xs), np.mean(ys)
   
    def Controller(self):

        if (
            self.DepthcamMessage is None or
            self.home_lat is None
        ):
            return
    
        #self.x = self.OdometryMessage.pose.pose.position.x
        #self.y = self.OdometryMessage.pose.pose.position.y
        
        self.x, self.y = self.global_to_local_xy(self.home_lat, self.home_lon, self.gps_msg.latitude, self.gps_msg.longitude)
        self.get_logger().info(f"Current Position: x={self.x}, y={self.y}")

        if self.compass_hdg is None:
            return

        
        compass_yaw = np.deg2rad(self.compass_hdg)

        
        theta = - compass_yaw 

        
        theta = np.arctan2(np.sin(theta), np.cos(theta))

 
        self.xd, self.yd = self.global_to_local_xy(
            self.home_lat,
            self.home_lon,
            self.goal_lat,
            self.goal_lon
        )
        self.get_logger().info(f"Goal Position: xd={self.xd}, yd={self.yd}")
        vector = np.array([[self.xd - self.x], [self.yd - self.y]])


        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.DepthcamMessage,
                desired_encoding='passthrough'
            )
 
            depth_mm = np.array(cv_image, dtype=np.uint16)
    
            depth_array = depth_mm.astype(np.float32) * 0.001
        except Exception as e:
            self.get_logger().error(str(e))
            return

        depth_array[depth_array == 0] = 100.0   


        center_row = depth_array.shape[0] // 2
        roi_height = 50
        depth_strip = depth_array[
            center_row - roi_height // 2 : center_row + roi_height // 2, :
        ]

        virtual_scan = np.min(depth_strip, axis=0)[::3]
        angles = np.linspace(
            self.fov_horizontal_rad / 2,
            -self.fov_horizontal_rad / 2,
            len(virtual_scan)
        )

        point_obstacles = []
        for d, a in zip(virtual_scan, angles):
            if self.range_min < d < self.range_max:
                point_obstacles.append(
                    (d *np.tan(a), d, d, a)
                )


        clusters = []
        current = []

        for p in point_obstacles:
            if not current:
                current.append(p)
                continue
            x1, y1, _, _ = current[-1]
            x2, y2, _, _ = p
            if np.hypot(x2 - x1, y2 - y1) < 0.75:
                current.append(p)
            else:
                if len(current) >= self.min_cluster_size:
                    clusters.append(current)
                current = [p]

        if current and len(current) >= self.min_cluster_size:
                clusters.append(current)
        
    
        
        Fa = self.ka * vector
        Fr = np.zeros((2, 1))

        for cluster in clusters:
            closest = min(cluster, key=lambda p: p[2])
            _, _, g, alpha = closest

            if g > self.gstar:
                continue

            mag = self.kr * (1.0 / g - 1.0 / self.gstar) / (g**3) #g**3 kardiya

            Fr -= np.array([
                [mag * np.cos(alpha + theta)],
                [mag * np.sin(alpha + theta)]
            ])

        #self.get_logger().info(f"Clusters detected: {len(clusters)}")
        cluster_len = Float64()
        cluster_len.data = float(len(clusters))
        self.cluster_publisher.publish(cluster_len)

        Fnet = Fa + Fr
        
        force_msg = Force()
        force_msg.fax = float(Fa[0])
        force_msg.fay = float(Fa[1])
        force_msg.frx = float(Fr[0])
        force_msg.fry = float(Fr[1])
        self.force_publisher.publish(force_msg)
        
        thetaD = np.arctan2(Fnet[1, 0], Fnet[0, 0])


        orientation = np.arctan2(
            np.sin(thetaD - theta),
            np.cos(thetaD - theta)
        )

        error_derivative = (orientation - self.prev_error_yaw) / self.dt
        
        thetavel = (self.kp_thetavel*orientation - self.kd_thetavel*error_derivative)
        
        self.prev_error_yaw = orientation
        thetavel = np.clip(thetavel,-0.15,0.15)


        if np.linalg.norm(vector) < self.eps_control:
            xvel = 0.0
            thetavel = 0.0
        else:
            xvel = 0.0 if abs(orientation) > self.eps_orient else min(np.linalg.norm(Fnet), 1.5)

        #self.get_logger().info(f"Vector norm: {np.linalg.norm(vector)}")        

        self.ControlVel.linear.x = float(xvel)
        self.ControlVel.angular.z = float(thetavel)
        self.VelocityPublisher.publish(self.ControlVel)



def main(args=None):

    rclpy.init(args=args)

    goal_lat = -35.36366071
    goal_lon = 149.16636646

    node = ControllerNode((goal_lat, goal_lon))
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()