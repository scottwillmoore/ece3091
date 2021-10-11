import cv2
import numpy as np
import rclpy
import time

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


def view_classification(net, image):
    ta = time.time()
    classes = ["BallBearing"]

    layer_names = net.getLayerNames()
    output_layers = [
        layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()
    ]

    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    font = cv2.FONT_HERSHEY_PLAIN

    frame = image

    height, width, channels = frame.shape
    # Detecting objects

    tb = time.time()
    print("tb - ta", tb - ta)

    blob = cv2.dnn.blobFromImage(
        frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False
    )

    tc = time.time()
    print("tc - tb", tc - tb)

    net.setInput(blob)
    outs = net.forward(output_layers)

    td = time.time()
    print("td - tc", td - tc)

    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.2:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    te = time.time()
    print("te - td", te - td)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)

    tf = time.time()
    print("tf - te", tf - te)

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = colors[class_ids[i]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.rectangle(frame, (x, y), (x + w, y + 30), color, -1)
            cv2.putText(
                frame,
                label + " " + str(round(confidence, 2)),
                (x, y + 30),
                font,
                3,
                (255, 255, 255),
                3,
            )

    tg = time.time()
    print("tg - tf", tg - tf)

    cv2.imshow("Image", frame)
    cv2.waitKey(1)

    elapsed = time.time() - ta
    print(elapsed)


class ComputerVisionNode(Node):
    def __init__(self):
        super().__init__("computer_vision")

        model = (
            self.declare_parameter("model", Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )

        if not model:
            raise Exception("The parameter 'model' is not set")

        self.get_logger().info(f"model = {model}")

        configuration = (
            self.declare_parameter("configuration", Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )

        if not configuration:
            raise Exception("The parameter 'configuration' is not set")

        self.get_logger().info(f"configuration = {configuration}")

        self.subscription = self.create_subscription(
            Image, "image", self.image_callback, 10
        )

        self.bridge = CvBridge()

        self.net = cv2.dnn.readNet(model, configuration)

    def image_callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message, "bgr8")

        view_classification(self.net, image)


def main(args=None):
    rclpy.init(args=args)
    computer_vision = ComputerVisionNode()
    rclpy.spin(computer_vision)
    computer_vision.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
