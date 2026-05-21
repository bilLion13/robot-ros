# ============================================================
# VISION NODE INDUSTRIEL
# ============================================================
#
# OBJECTIF :
#
# Détecter des zones potentiellement dangereuses
# dans un environnement industriel :
#
#   • trous
#   • fissures
#   • ouvertures
#   • zones sombres anormales
#   • anomalies au sol
#
# Cette version est plus réaliste qu’une détection
# basée uniquement sur des cercles.
#
# Le système utilise :
#
#   • segmentation sombre
#   • contours OpenCV
#   • analyse d’aire
#   • analyse de contraste
#   • validation multi-frames
#
# ============================================================
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool

import cv2
import numpy as np


# ============================================================
# PARAMÈTRES DE DÉTECTION
# ============================================================

DARK_THRESHOLD = 85
MIN_AREA = 1500
MIN_CONTRAST = 18
CONFIRM_THRESHOLD = 1


# ============================================================
# PARAMÈTRE CAMÉRA
# ============================================================
# servo_s1 = horizontal gauche/droite
# servo_s2 = vertical haut/bas
#
# On ne bouge plus servo_s1.
# On garde seulement la caméra légèrement levée avec servo_s2.

CAMERA_VERTICAL_ANGLE = -45


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ====================================================
        # CAMÉRA
        # ====================================================

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Caméra non détectée")
        else:
            self.get_logger().info("Caméra OK")

        # ====================================================
        # PUBLISHERS
        # ====================================================

        # Servo vertical seulement
        self.servo2_pub = self.create_publisher(
            Int32,
            '/servo_s2',
            10
        )

        # Détection danger
        self.danger_pub = self.create_publisher(
            Int32,
            '/danger_detected',
            10
        )

        # ====================================================
        # MODE AUTO / MANUEL
        # ====================================================

        self.sub_joy = self.create_subscription(
            Bool,
            '/JoyState',
            self.joystate_callback,
            10
        )

        self.manual_mode = True

        # ====================================================
        # CONFIRMATION MULTI-FRAMES
        # ====================================================

        self.confirm_count = 0

        # ====================================================
        # TIMERS
        # ====================================================

        self.timer = self.create_timer(
            0.08,
            self.process
        )

        # Garde la caméra levée régulièrement
        self.camera_timer = self.create_timer(
            1.0,
            self.publish_camera_position
        )

        self.publish_camera_position()

        self.get_logger().info(
            "VISION NODE READY → caméra fixe levée + /danger_detected"
        )

    # ========================================================
    # POSITION CAMÉRA
    # ========================================================

    def publish_camera_position(self):
        """
        Garde la caméra légèrement levée.
        On utilise uniquement /servo_s2.
        Aucun mouvement gauche/droite.
        """
        # NE PAS contrôler la caméra en manuel
        if self.manual_mode:
            return

        msg = Int32()
        msg.data = int(CAMERA_VERTICAL_ANGLE)
        self.servo2_pub.publish(msg)

    # ========================================================
    # MODE AUTO / MANUEL
    # ========================================================

    def joystate_callback(self, msg: Bool):
        self.manual_mode = msg.data

        state = "MANUEL" if self.manual_mode else "AUTO"

        self.get_logger().info(f"Mode : {state}")

        if not self.manual_mode:
            self.publish_camera_position()

    # ========================================================
    # PUBLICATION DANGER
    # ========================================================

    def publish_danger(self, value: int):
        msg = Int32()
        msg.data = int(value)
        self.danger_pub.publish(msg)

    # ========================================================
    # VALIDATION D'UNE ZONE DANGEREUSE
    # ========================================================

    def validate_danger_zone(self, gray, contour):
        h, w = gray.shape

        area = cv2.contourArea(contour)

        if area < MIN_AREA:
            return False, "petit"

        x, y, cw, ch = cv2.boundingRect(contour)

        # Ignore les objets trop hauts dans l'image
        if y < h * 0.25:
            return False, "trop haut"

        mask = np.zeros(gray.shape, dtype=np.uint8)

        cv2.drawContours(
            mask,
            [contour],
            -1,
            255,
            -1
        )

        mean_inside = cv2.mean(
            gray,
            mask=mask
        )[0]

        if mean_inside > DARK_THRESHOLD:
            return False, f"clair({mean_inside:.0f})"

        dilated = cv2.dilate(
            mask,
            np.ones((15, 15), np.uint8),
            iterations=1
        )

        ring = cv2.subtract(dilated, mask)

        mean_outside = cv2.mean(
            gray,
            mask=ring
        )[0]

        contrast = mean_outside - mean_inside

        if contrast < MIN_CONTRAST:
            return False, f"contraste({contrast:.0f})"

        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)

        if hull_area == 0:
            return False, "hull"

        solidity = area / hull_area

        if solidity < 0.35:
            return False, f"forme({solidity:.2f})"

        return True, f"DANGER a={int(area)} c={contrast:.0f} s={solidity:.2f}"

    # ========================================================
    # TRAITEMENT IMAGE
    # ========================================================

    def process(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Image non reçue")
            return

        frame = cv2.resize(frame, (640, 480))
        debug_frame = frame.copy()

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        blurred = cv2.GaussianBlur(
            gray,
            (9, 9),
            0
        )

        _, thresh = cv2.threshold(
            blurred,
            DARK_THRESHOLD,
            255,
            cv2.THRESH_BINARY_INV
        )

        kernel = np.ones((5, 5), np.uint8)

        thresh = cv2.morphologyEx(
            thresh,
            cv2.MORPH_CLOSE,
            kernel
        )

        thresh = cv2.morphologyEx(
            thresh,
            cv2.MORPH_OPEN,
            kernel
        )

        contours, _ = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        danger_found = False

        for contour in contours:
            valid, reason = self.validate_danger_zone(
                gray,
                contour
            )

            color = (0, 255, 0) if valid else (0, 0, 255)

            cv2.drawContours(
                debug_frame,
                [contour],
                -1,
                color,
                2
            )

            M = cv2.moments(contour)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            cv2.circle(
                debug_frame,
                (cx, cy),
                4,
                color,
                -1
            )

            x, y, w, h = cv2.boundingRect(contour)

            cv2.putText(
                debug_frame,
                reason,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1
            )

            if valid:
                self.confirm_count += 1

                if self.confirm_count >= CONFIRM_THRESHOLD:
                    danger_found = True
                    self.publish_danger(1)

                    self.get_logger().info(
                        f"DANGER DETECTE "
                        f"({self.confirm_count}x) "
                        f"X={cx} Y={cy}"
                    )

            else:
                self.confirm_count = 0

        if not danger_found:
            self.confirm_count = 0
            self.publish_danger(0)

        mode = "MANUEL" if self.manual_mode else "AUTO"

        cv2.putText(
            debug_frame,
            f"Mode:{mode} Camera fixe S2:{CAMERA_VERTICAL_ANGLE}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2
        )

        cv2.putText(
            debug_frame,
            f"Danger:{1 if danger_found else 0}",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2
        )

        cv2.putText(
            debug_frame,
            f"Confirm:{self.confirm_count}/{CONFIRM_THRESHOLD}",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2
        )

        cv2.imshow("Camera", debug_frame)
        cv2.imshow("Threshold", thresh)
        cv2.waitKey(1)


def main():
    rclpy.init()

    node = VisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()