#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


# ==== CONSTANTES FIJAS ====
IMAGE_TOPIC = '/camera1_image'
COLOR_TOPIC = '/color_detected'
UMBRAL_PIXELS = 100

LOWER_GREEN = np.array([35, 50, 50])
UPPER_GREEN = np.array([85, 255, 255])


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Int32, COLOR_TOPIC, 10)
        self.sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f"Suscrito a {IMAGE_TOPIC}, publicando en {COLOR_TOPIC}")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Error en cv_bridge: {e}")
            return

        # Conversión a HSV y segmentación por color
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        cv2.imshow("Color mask", mask)
        cv2.waitKey(1)

        # Contar píxeles y publicar si supera el umbral
        pixels_detectados = int(cv2.countNonZero(mask))
        if pixels_detectados > UMBRAL_PIXELS:
            print(f"Detectados: {pixels_detectados}")
            self.pub.publish(Int32(data=pixels_detectados))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
