#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest

# --- Параметры роя ---
UAV_NAMES = ['uav0', 'uav1', 'uav2']  # Соответствует пространствам имен в multi_uav_mavros_sitl.launch
LEADER_NAME = 'uav0'
FORMATION_OFFSETS = {
    'uav0': (0.0, 0.0, 0.0), # Лидер: смещение 0
    'uav1': (-2.0, 0.0, 0.0), # Ведомый 1: 2 метра позади
    'uav2': (-1.0, -1.5, 0.0)  # Ведомый 2: Сзади и справа
}
TARGET_ALTITUDE = 5.0 # Желаемая высота полета

class SwarmController:
    def __init__(self):
        rospy.init_node('swarm_controller_node', anonymous=True)
        self.rate = rospy.Rate(20.0) # Частота цикла управления (20 Hz)
        self.uav_states = {}
        self.uav_positions = {}
        self.uav_setpoints = {}
        self.service_proxies = {}

        for name in UAV_NAMES:
            self.uav_states[name] = State()
            self.uav_positions[name] = PoseStamped()
            
            # Инициализация Publisher для setpoints
            self.uav_setpoints[name] = rospy.Publisher(f'/{name}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            
            # 1. Подписки (Телеметрия)
            rospy.Subscriber(f'/{name}/mavros/state', State, self.state_cb, name)
            rospy.Subscriber(f'/{name}/mavros/local_position/pose', PoseStamped, self.pose_cb, name)
            
            # 2. Сервисы (Arming и SetMode)
            rospy.wait_for_service(f'/{name}/mavros/cmd/arming')
            rospy.wait_for_service(f'/{name}/mavros/set_mode')
            self.service_proxies[name] = {
                'arming': rospy.ServiceProxy(f'/{name}/mavros/cmd/arming', CommandBool),
                'set_mode': rospy.ServiceProxy(f'/{name}/mavros/set_mode', SetMode)
            }
        
        rospy.loginfo("Swarm Controller Initialized and connected to all MAVROS nodes.")

    def state_cb(self, msg, uav_name):
        self.uav_states[uav_name] = msg

    def pose_cb(self, msg, uav_name):
        self.uav_positions[uav_name] = msg

    def set_mode(self, uav_name, mode):
        """Переключает режим полета."""
        req = SetModeRequest()
        req.custom_mode = mode
        try:
            response = self.service_proxies[uav_name]['set_mode'](req)
            if response.mode_sent:
                rospy.loginfo(f"{uav_name}: Mode changed to {mode}")
            # else: # Частота в 20Гц может привести к спаму логов, если режим не установлен сразу
            #     rospy.logwarn(f"{uav_name}: Failed to set mode to {mode}")
            return response.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"{uav_name} SetMode failed: {e}")
            return False

    def arm_vehicle(self, uav_name):
        """Вооружает (Arm) БПЛА."""
        req = CommandBoolRequest()
        req.value = True
        try:
            response = self.service_proxies[uav_name]['arming'](req)
            if response.success:
                rospy.loginfo(f"{uav_name}: Vehicle armed successfully.")
            # else:
            #     rospy.logwarn(f"{uav_name}: Arming failed.")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"{uav_name} Arming service call failed: {e}")
            return False

    def get_setpoint_pose(self, x, y, z):
        """Создает объект PoseStamped для публикации."""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Ориентацию можно добавить здесь, но для простого удержания позиции это не обязательно.
        return pose

    def publish_setpoints(self, uav_name, pose_stamped):
        """Публикует желаемую позицию для конкретного БПЛА."""
        pose_stamped.header.stamp = rospy.Time.now()
        self.uav_setpoints[uav_name].publish(pose_stamped)

    def run(self):
        # Ожидание первого получения состояния MAVROS
        for name in UAV_NAMES:
            rospy.loginfo(f"Waiting for {name} MAVROS connection...")
            while(not rospy.is_shutdown() and not self.uav_states[name].connected):
                # Публикация нулевого setpoint для обхода проверки Offboard (PX4 требует setpoints)
                self.publish_setpoints(name, self.get_setpoint_pose(0, 0, 0))
                self.rate.sleep()
        rospy.loginfo("All MAVROS nodes connected.")
        
        # --- ФАЗА 1: Установка режима Offboard и Arming ---
        offboard_ready = False
        arm_attempt_count = 0
        max_arm_attempts = 100 
        
        while not rospy.is_shutdown() and not offboard_ready and arm_attempt_count < max_arm_attempts:
            ready_count = 0
            for name in UAV_NAMES:
                # 1. Постоянно публикуем setpoints (требование для включения Offboard)
                self.publish_setpoints(name, self.get_setpoint_pose(0, 0, 0))

                # 2. Устанавливаем режим Offboard
                if self.uav_states[name].mode != "OFFBOARD":
                    self.set_mode(name, "OFFBOARD")

                # 3. Вооружаем (Arm)
                if not self.uav_states[name].armed:
                    self.arm_vehicle(name)
                
                if self.uav_states[name].armed and self.uav_states[name].mode == "OFFBOARD":
                    ready_count += 1
            
            arm_attempt_count += 1

            if ready_count == len(UAV_NAMES):
                offboard_ready = True
                rospy.loginfo("All UAVs are ARMED and in OFFBOARD mode. Starting coordinated flight.")

            self.rate.sleep()
        
        if not offboard_ready:
            rospy.logerr("Failed to set all UAVs to OFFBOARD mode and arm after multiple attempts. Check MAVROS/PX4 logs.")
            return

        # --- ФАЗА 2: Координированный полет ---
        flight_path = [
            (0.0, 0.0, TARGET_ALTITUDE), # Взлет на заданную высоту (Home position)
            (5.0, 0.0, TARGET_ALTITUDE), # Точка 1: Вперед
            (5.0, 5.0, TARGET_ALTITUDE), # Точка 2: Вправо
            (0.0, 5.0, TARGET_ALTITUDE), # Точка 3: Назад
            (0.0, 0.0, 2.0),             # Точка 4: Снижение перед посадкой
        ]
        
        waypoint_idx = 0
        waypoint_tolerance = 0.5 # Допустимое отклонение от точки (в метрах)

        while not rospy.is_shutdown():
            if waypoint_idx < len(flight_path):
                target_x, target_y, target_z = flight_path[waypoint_idx]
                
                # --- ЛОГИКА ГЛАВНОГО КОПТЕРА (uav0) ---
                # uav0 преследует текущую целевую точку
                leader_sp_pose = self.get_setpoint_pose(target_x, target_y, target_z)
                self.publish_setpoints(LEADER_NAME, leader_sp_pose)

                # Проверка, достиг ли лидер целевой точки
                leader_pos = self.uav_positions[LEADER_NAME].pose.position
                dist_to_target = math.sqrt(
                    (leader_pos.x - target_x)**2 + 
                    (leader_pos.y - target_y)**2 + 
                    (leader_pos.z - target_z)**2
                )

                if dist_to_target < waypoint_tolerance:
                    rospy.loginfo(f"Leader reached waypoint {waypoint_idx}: ({target_x}, {target_y}, {target_z}). Moving to next.")
                    waypoint_idx += 1
                
                # --- ЛОГИКА ВЕДОМЫХ КОПТЕРОВ (uav1, uav2) ---
                # Ведомые преследуют желаемую позицию, рассчитанную относительно лидера
                
                # Берем setpoint лидера (не его текущую позицию)
                leader_target_x = target_x
                leader_target_y = target_y
                leader_target_z = target_z

                for name in UAV_NAMES:
                    if name != LEADER_NAME:
                        # Применяем статическое смещение к желаемой позиции лидера (Coordination)
                        offset_x, offset_y, offset_z = FORMATION_OFFSETS[name]
                        
                        follower_target_x = leader_target_x + offset_x
                        follower_target_y = leader_target_y + offset_y
                        follower_target_z = leader_target_z + offset_z
                        
                        follower_sp_pose = self.get_setpoint_pose(follower_target_x, follower_target_y, follower_target_z)
                        self.publish_setpoints(name, follower_sp_pose)
                        
            else:
                # Маршрут завершен: Посадка (Switch to LAND mode)
                rospy.loginfo("Flight complete. Attempting to switch all UAVs to AUTO.LAND mode.")
                
                all_landed = True
                for name in UAV_NAMES:
                    if self.uav_states[name].mode != "AUTO.LAND":
                         self.set_mode(name, "AUTO.LAND")
                    
                    # Проверяем, что коптер не вооружен (один из признаков, что он приземлился)
                    if self.uav_states[name].armed:
                        all_landed = False

                if all_landed:
                    rospy.loginfo("All UAVs appear to be landed and disarmed. Controller shutting down.")
                    break # Выход из цикла run
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SwarmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
