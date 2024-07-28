import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from parc_robot_interfaces.msg import CropYield
from std_msgs.msg import String
from .perceiver import AgribotPerceiver, SensorType
from .centroid_tracker import CentroidTracker
from .constants import DEFAULT_QoS_PROFILE_VALUE, AGRIBOT_YE_NODE_NAME


class AgribotCropYieldEstimator(Node):
    def __init__(
        self,
        show_images: bool = True,
    ) -> None:
        super().__init__(AGRIBOT_YE_NODE_NAME)
        self.robot_finished = False
        self.show_images = show_images
        self.perceptor = AgribotPerceiver(self)

        self.left_tracker = CentroidTracker(10)
        self.right_tracker = CentroidTracker(10)

        self.create_subscription(
            String,
            "/parc_robot/robot_status",
            self.status_callback,
            DEFAULT_QoS_PROFILE_VALUE,
        )

        # self.perceptor.register_state_update_listener(
        #     sensor_type=SensorType.RIGHT_CAM,
        #     callback=self.image_right_callback,
        # )

        self.perceptor.register_state_update_listener(
            sensor_type=SensorType.LEFT_CAM,
            callback=self.image_left_callback,
        )

        self.crop_yield_pub = self.create_publisher(
            CropYield,
            "/parc_robot/crop_yield",
            DEFAULT_QoS_PROFILE_VALUE,
        )

        self.robot_status_sub = self.create_subscription(
            String,
            "/parc_robot/robot_status",
            self.status_callback,
            DEFAULT_QoS_PROFILE_VALUE,
        )

    @property
    def total_tracked(self) -> int:
        return self.left_tracker.nextObjectID + self.right_tracker.nextObjectID

    def image_left_callback(self, image):
        if not self.robot_finished:
            self.process_image(image, "left")

    def image_right_callback(self, image):
        if not self.robot_finished:
            self.process_image(image, "right")

    def process_image(self, image, camera_name):
        try:
            self.detect_red_fruits(image, camera_name)
            tracker = self.left_tracker if camera_name == "left" else self.right_tracker
            self.get_logger().info(
                f"Current frame fruit count from {camera_name} camera: {tracker.nextObjectID}"
            )
            if self.show_images:
                # Exibir a imagem com as frutas detectadas
                self.display_image(image, camera_name)
        except Exception as e:
            self.get_logger().error(
                f"Error processing image from {camera_name} camera: {e}"
            )

    def detect_red_fruits(self, image, camera_name):

        # Definir a altura limite para a máscara
        #height_limit = 200  # Ajuste conforme necessário

        # Criar uma máscara do mesmo tamanho que a imagem
        #mask = np.zeros(image.shape[:2], dtype=np.uint8)

        # Preencher a parte inferior da máscara com branco (1)
        #mask[height_limit:, :] = 255

        # Aplicar a máscara na imagem para a detecção
        # Usando a máscara para limitar a área de detecção
        #image_masked = cv2.bitwise_and(image, image, mask=mask)
        # Convertendo a imagem de BGR para HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definir os limites de cor para os tomates (vermelho)
        lower_red1 = np.array([160, 100, 100])
        upper_red1 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

        lower_red2 = np.array([170, 50, 30])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        mask = mask1 | mask2

        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Aplicar filtro de desfoque para reduzir o ruído
        blurred = cv2.GaussianBlur(mask, (9, 9), 10)

        # Detectar círculos usando a Transformada de Hough
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=20,
            param1=100,
            param2=20,
            minRadius=10,
            maxRadius=50,
        )

        rects = []

        # Se círculos forem detectados
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                start_x, start_y = i[0] - i[2], i[1] - i[2]
                end_x, end_y = start_x + 2 * i[2], start_y + 2 * i[2]
                cv2.rectangle(image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
                rects.append(np.array((start_x, start_y, end_x, end_y), dtype="int"))

        objects = None

        if camera_name == "left":
            objects = self.left_tracker.update(rects)
        elif camera_name == "right":
            objects = self.right_tracker.update(rects)

        if objects:
            for objectID, centroid in objects.items():
                # draw both the ID of the object and the centroid of the
                # object on the output frame
                text = "ID {}".format(objectID)
                cv2.putText(
                    image,
                    text,
                    (centroid[0] - 10, centroid[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                cv2.circle(image, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

    def kmeans_segmentation(
        self,
        image,
        attempts=10,
        k=4,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0),
    ):
        td_img = np.float32(image.reshape((-1, 3)))
        _, label, center = cv2.kmeans(
            td_img, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS
        )

        center = np.uint8(center)
        res = center[label.flatten()]

        return res.reshape((image.shape))

    def display_image(self, image, camera_name):
        # Colocar a contagem de frutas na imagem
        cv2.putText(
            image,
            f"Total Fruits: {self.total_tracked}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        cv2.imshow(f"{camera_name} camera", image)
        cv2.waitKey(1)

    def status_callback(self, msg):
        if msg.data == "finished" and not self.robot_finished:
            self.robot_finished = True
            self.publish_yield()

    def publish_yield(self):
        self.log_total()
        msg = CropYield()
        msg.data = self.total_tracked
        self.crop_yield_pub.publish(msg)

    def log_total(self):
        self.get_logger().info(f"Final total crop yield: {self.total_tracked}")


def main(args=None):
    rclpy.init(args=args)
    try:
        yield_estimator = AgribotCropYieldEstimator()
        rclpy.spin(yield_estimator)
        rclpy.shutdown()
    except KeyboardInterrupt as e:
        pass
    except Exception as e:
        print("======> Ended with exception:", e.with_traceback())
    finally:
        yield_estimator.log_total()
        yield_estimator.destroy_node()


if __name__ == "__main__":
    main()
