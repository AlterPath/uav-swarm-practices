#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageDisplayNode:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.window_name = "ROS Image Subscriber"
        
        # Создаем окно для отображения
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        # Подписываемся на тему с изображениями
        rospy.Subscriber('image_topic', Image, self.image_callback)
        
        # Переменная для хранения текущего изображения
        self.current_image = None
        
    def image_callback(self, msg):
        try:
            # Конвертируем ROS Image message в numpy array
            image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Сохраняем текущее изображение
            self.current_image = image_np
            
            # Выводим информацию о изображении
            print(f"Получено изображение: {image_np.shape}, dtype: {image_np.dtype}")
            print(f"Средняя яркость: {np.mean(image_np):.2f}")
            
        except Exception as e:
            print(f"Ошибка конвертации: {e}")
    
    def run(self):
        rate = rospy.Rate(30)  # 30 FPS для плавного отображения
        
        while not rospy.is_shutdown():
            # Если есть изображение - отображаем его
            if self.current_image is not None:
                cv2.imshow(self.window_name, self.current_image)
            
            # Обрабатываем события OpenCV (нужно для работы окон)
            key = cv2.waitKey(1) & 0xFF
            
            # Выход по нажатию 'q' или ESC
            if key == ord('q') or key == 27:
                break
                
            rate.sleep()
        
        # Закрываем окно при завершении
        cv2.destroyAllWindows()

def simple_image_subscriber():
    """Упрощенная версия без класса"""
    rospy.init_node('image_subscriber', anonymous=True)
    bridge = CvBridge()
    window_name = "ROS Image Subscriber"
    
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    def image_callback(msg):
        try:
            image_np = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Немедленно отображаем изображение
            cv2.imshow(window_name, image_np)
            cv2.waitKey(1)  # Необходимо для обновления окна
            
            print(f"Изображение: {image_np.shape}, яркость: {np.mean(image_np):.2f}")
            
        except Exception as e:
            print(f"Ошибка: {e}")
    
    rospy.Subscriber('image_topic', Image, image_callback)
    
    # Ожидаем завершения
    rospy.spin()
    
    # Закрываем окно при завершении
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Используем класс для лучшей организации кода
    node = ImageDisplayNode()
    
    # Или используем простую версию:
    # simple_image_subscriber()
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
