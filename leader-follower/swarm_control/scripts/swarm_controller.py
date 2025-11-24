#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class Quadrotor:
    def __init__(self, namespace):
        self.ns = namespace
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Publishers
        self.local_pos_pub = rospy.Publisher(self.ns + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber(self.ns + '/mavros/state', State, self.state_cb)
        self.local_pos_sub = rospy.Subscriber(self.ns + '/mavros/local_position/pose', PoseStamped, self.pose_cb)
        
    def state_cb(self, data):
        self.current_state = data

    def pose_cb(self, data):
        self.current_pose = data

    def set_position(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_footprint"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.local_pos_pub.publish(pose)

class SwarmController:
    def __init__(self):
        rospy.init_node('swarm_coordinated_control', anonymous=True)
        
        self.rate = rospy.Rate(20)
        
        # Инициализация квадрокоптеров
        self.leader = Quadrotor('/uav0')
        self.follower1 = Quadrotor('/uav1') 
        self.follower2 = Quadrotor('/uav2')
        
        # Параметры формации
        self.formation_distance = 2.0
        self.flight_altitude = 3.0
        
        # Параметры траектории
        self.circle_radius = 5.0
        self.angular_speed = 0.3
        self.angle = 0.0
        
        # Ожидание подключения
        self.wait_for_connection()

    def wait_for_connection(self):
        rospy.loginfo("Waiting for FCU connection...")
        for quad in [self.leader, self.follower1, self.follower2]:
            while not rospy.is_shutdown() and not quad.current_state.connected:
                self.rate.sleep()
        rospy.loginfo("All FCUs connected")

    def set_mode_offboard(self):
        rospy.loginfo("Setting OFFBOARD mode...")
        for quad in [self.leader, self.follower1, self.follower2]:
            rospy.wait_for_service(quad.ns + '/mavros/set_mode')
            try:
                set_mode = rospy.ServiceProxy(quad.ns + '/mavros/set_mode', SetMode)
                response = set_mode(0, 'OFFBOARD')
                rospy.loginfo(f"Set mode for {quad.ns}: {response}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Set mode failed for {quad.ns}: {e}")

    def arm_all(self):
        rospy.loginfo("Arming vehicles...")
        for quad in [self.leader, self.follower1, self.follower2]:
            rospy.wait_for_service(quad.ns + '/mavros/cmd/arming')
            try:
                arming = rospy.ServiceProxy(quad.ns + '/mavros/cmd/arming', CommandBool)
                response = arming(True)
                rospy.loginfo(f"Arming {quad.ns}: {response}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Arming failed for {quad.ns}: {e}")

    def takeoff_sequence(self):
        rospy.loginfo("Starting takeoff sequence...")
        
        # Отправка setpoint'ов перед взлетом
        for i in range(100):
            if rospy.is_shutdown():
                break
                
            self.leader.set_position(0, 0, self.flight_altitude)
            self.follower1.set_position(-self.formation_distance, 0, self.flight_altitude)
            self.follower2.set_position(0, self.formation_distance, self.flight_altitude)
            self.rate.sleep()
        
        # Установка режима и арминг
        self.set_mode_offboard()
        self.arm_all()
        
        # Дополнительное ожидание
        rospy.sleep(2)

    def calculate_formation_positions(self, leader_x, leader_y):
        # Ведомые располагаются треугольником относительно лидера
        follower1_x = leader_x - self.formation_distance
        follower1_y = leader_y
        
        follower2_x = leader_x + self.formation_distance * math.cos(math.pi/3)
        follower2_y = leader_y + self.formation_distance * math.sin(math.pi/3)
        
        return follower1_x, follower1_y, follower2_x, follower2_y

    def run(self):
        self.takeoff_sequence()
        rospy.loginfo("Starting coordinated swarm control...")
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Круговая траектория лидера
            current_time = (rospy.Time.now() - start_time).to_sec()
            self.angle = self.angular_speed * current_time
            
            leader_x = self.circle_radius * math.cos(self.angle)
            leader_y = self.circle_radius * math.sin(self.angle)
            
            # Расчет позиций ведомых
            f1_x, f1_y, f2_x, f2_y = self.calculate_formation_positions(leader_x, leader_y)
            
            # Отправка целевых позиций
            self.leader.set_position(leader_x, leader_y, self.flight_altitude)
            self.follower1.set_position(f1_x, f1_y, self.flight_altitude)
            self.follower2.set_position(f2_x, f2_y, self.flight_altitude)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SwarmController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Swarm controller terminated")
