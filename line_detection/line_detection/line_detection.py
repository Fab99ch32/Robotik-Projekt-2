import rclpy
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError


class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')

        # Abonnieren des Kamera-Images
        self.image_sub = self.create_subscription(Image, 'camera_image', self.image_callback, 10)

        # Publisher für Distanz und Winkel
        self.distance_pub = self.create_publisher(Float64, 'distance_to_optical_axis', 10)
        self.angle_pub = self.create_publisher(Float64, 'line_angle', 10)

        self.bridge = CvBridge()

        # Timer für periodische Aktualisierung
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1

    def image_callback(self, data):
        try:
            # Konvertiere ROS-Image nach OpenCV-Image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Funktion zum Umwandeln des Bildes in ein Binärbild
            def convert_to_binary(image):
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                _, binary_image = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
                return binary_image


            # Funktion zur Durchführung der Segmentierung
            def segmentation(image):
                contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                segmented_image = np.zeros_like(image)
                cv2.drawContours(segmented_image, contours, -1, (255), thickness=cv2.FILLED)
                return segmented_image, contours

            # Funktion zur Durchführung der morphologischen Transformation
            def morphological_transform(image):
                kernel = np.ones((5, 5), np.uint8)
                transformed_image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
                return transformed_image

            # Funktion zum Abschneiden der unteren zwei Drittel des Bildes
            def crop_lower_two_thirds(image, cut_percentage_top=50, cut_percentage_bottom=10):
                height, width = image.shape[:2]
                upper_percentage = cut_percentage_top
                lower_percentage = cut_percentage_bottom

                upper_limit = int(upper_percentage * height / 100)
                lower_limit = height - int(lower_percentage * height / 100)

                return image[upper_limit:lower_limit, :]

            


            cropped_image = crop_lower_two_thirds(image)
            smoothed_image = cv2.GaussianBlur(cropped_image, (7, 7), 0)

            # 2. Umwandeln des Bildes in ein Binärbild
            binary_image = convert_to_binary(smoothed_image)

            # 4. Durchführung der morphologischen Transformation
            transformed_image = morphological_transform(binary_image)
            smoothed_image = cv2.GaussianBlur(transformed_image, (7, 7), 0)

            # 3. Durchführung der Segmentierung und Erhalt der Konturen
            segmented_image, contours = segmentation(smoothed_image)

            # Sortiere Konturen nach ihrer Fläche in absteigender Reihenfolge
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            
            try:
                if (contours[0] is not None):

                    # Wähle die größte Kontur
                    largest_contour = contours[0]

                    # Falls eine weiße Kontur gefunden wurde
                    
                    # Approximiere die Kontur, um die Eckpunkte zu erhalten
                    epsilon = 0.02 * cv2.arcLength(largest_contour, True)
                    approximated_contour = cv2.approxPolyDP(largest_contour, epsilon, True)

                    # Berechne und veröffentliche Distanz und Winkel
                    distance, angle = self.calculate_distance_and_angle(approximated_contour, cropped_image, image)
                    if (distance and angle) is not None:
                        self.distance_pub.publish(Float64(data=distance))
                        self.angle_pub.publish(Float64(data=angle))

            
            except Exception as  e:
                self.get_logger().info('Fehler in Berechnung')

        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def calculate_distance_and_angle(self, approximated_contour, cropped_image, image):
        def get_corners(approximated_contour):
            # Sortiere die Ecken nach ihrer x-Koordinate
            sorted_corners = sorted(approximated_contour, key=lambda x: x[0][0])

            if len(sorted_corners) == 3:
                # Nur drei Ecken gefunden, oben rechts wird basierend auf den beiden unteren Ecken berechnet
                top_right_x = sorted_corners[1][0][0] + (sorted_corners[2][0][0] - sorted_corners[1][0][0]) / 2
                top_right_y = sorted_corners[1][0][1] + (sorted_corners[2][0][1] - sorted_corners[1][0][1]) / 2
                top_right = np.array([[top_right_x, top_right_y]], dtype=np.int32)

                # Unten links: Ecke mit der kleinsten Summe von x und y
                bottom_left = min(sorted_corners, key=lambda x: x[0][0] + x[0][1])

                return sorted_corners[0], top_right, bottom_left, sorted_corners[1]
            else:
                # Vier Ecken gefunden
                # Unten links: Ecke mit der kleinsten Summe von x und y
                bottom_left = min(sorted_corners, key=lambda x: x[0][0] + x[0][1])

                # Unten rechts: Ecke mit der größten Differenz von x und y
                bottom_right = max(sorted_corners, key=lambda x: x[0][0] - x[0][1])

                # Oben links: Ecke mit der größten Summe von x und y
                top_left = max(sorted_corners, key=lambda x: x[0][0] + x[0][1])

                # Oben rechts: Ecke mit der größten Differenz von x und y
                top_right = min(sorted_corners, key=lambda x: x[0][0] - x[0][1])

                return top_left, top_right, bottom_left, bottom_right
        def calculate_midpoints(corners):
            # Berechne die Mittelpunkte der gegenüberliegenden Ecken
            midpoint_bottom = ((corners[0][0][0] + corners[1][0][0]) / 2, (corners[0][0][1] + corners[1][0][1]) / 2)
            midpoint_top = ((corners[2][0][0] + corners[3][0][0]) / 2, (corners[2][0][1] + corners[3][0][1]) / 2)

            return midpoint_bottom, midpoint_top

        try:
            if approximated_contour[3][0][0] is not None:
                corners = get_corners(approximated_contour)

                # Berechne die Mittelpunkte
                midpoint_bottom, midpoint_top = calculate_midpoints(corners)

                angle_middle = np.degrees(np.arctan2(midpoint_top[1] - midpoint_bottom[1],
                                                      midpoint_top[0] - midpoint_bottom[0]))
                angle_middle = abs(angle_middle)

            elif approximated_contour[0][0][0] is not None:
                corners = get_corners(approximated_contour)

                # Berechne die Mittelpunkte
                midpoint_bottom, midpoint_top = calculate_midpoints(corners)

                angle_middle = np.degrees(np.arctan2(midpoint_top[1] - midpoint_bottom[1],
                                                      midpoint_top[0] - midpoint_bottom[0]))
                
                angle_middle = abs(angle_middle)

            else:
                angle_middle = None

            
            
            
            self.get_logger().info(f'Winkel: {angle_middle}')

            # Abstandsberechnung

            # Ursprüngliche Bildhöhe und Breite
            original_height, original_width = image.shape[:2]

            # Umrechnung der Koordinaten in die ursprünglichen Bildkoordinaten
            original_mittlepunkt_unten_x = midpoint_bottom[0]
            original_mittlepunkt_unten_y = midpoint_bottom[1] + (original_height - cropped_image.shape[0])

            original_mittlepunkt_oben_x = midpoint_top[0]
            original_mittlepunkt_oben_y = midpoint_top[1] + (original_height - cropped_image.shape[0])

            


            
            slope = (original_mittlepunkt_oben_y - original_mittlepunkt_unten_y) / (
                original_mittlepunkt_oben_x - original_mittlepunkt_unten_x)
            intercept = original_mittlepunkt_oben_y - slope * original_mittlepunkt_oben_x

            
            # Höhe y = 300 in den ursprünglichen Bildkoordinaten
            y_value = 654  # 384



            if slope != 0:
                # Berechnung der x-Koordinate auf Höhe y = 300 in den ursprünglichen Bildkoordinaten
                x_at_y_300_original = (y_value - intercept) / slope
                Entfernung_optische_Achse = (original_width/ 2 - 35) - x_at_y_300_original
            else:
                # Handle den Fall, wenn die Linie vertikal ist (slope = 0)
                x_at_y_300_original = (original_width / 2 - 35)

        
        


            # Entfernung zum Mittelpunkt
            Entfernung_optische_Achse = (image.shape[1] / 2 - 35) - x_at_y_300_original
            self.get_logger().info(f'Entfernung: {Entfernung_optische_Achse}')
            self.get_logger().info(f'Steigung: {slope}')
            self.get_logger().info(f'x koordinate: {x_at_y_300_original}')

            return Entfernung_optische_Achse, angle_middle
        except Exception as e:
            self.get_logger().info(f'Fehler in Objekterkennung: {str(e)}')
            Entfernung_optische_Achse = 0.0
            angle_middle = 90.0
            return Entfernung_optische_Achse, angle_middle


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()