#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image # Tùy chọn, nếu muốn dùng ảnh
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose # Import ObjectHypothesisWithPose để dễ đọc code

class scene_recognition(Node):
    def __init__(self):
        super().__init__('scene_recognition')

        # --- Parameters ---
        self.declare_parameter('detection_topic', '/oak/nn/spatial_detections')
        # self.declare_parameter('image_topic', '/oak/rgb/image_raw') # Tùy chọn: Topic hình ảnh
        self.declare_parameter('output_scene_topic', '/inferred_scene')
        self.declare_parameter('confidence_threshold', 0.5)

        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        # image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_scene_topic = self.get_parameter('output_scene_topic').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        self.get_logger().info(f"Subscribing to detections on: {detection_topic}")
        # self.get_logger().info(f"Subscribing to image on: {image_topic}") # Tùy chọn
        self.get_logger().info(f"Publishing inferred scene to: {output_scene_topic}")
        self.get_logger().info(f"Using confidence threshold: {self.confidence_threshold}")

        # --- Define Object ID to Name Map ---
        # This map depends on the specific NN model used by your camera.
        # You MUST replace this with the actual mapping from your model/driver documentation.
        # This is a common map for models trained on the COCO dataset.
        self.ID_TO_NAME_MAP = {
            '1': 'person', '2': 'bicycle', '3': 'car', '4': 'motorcycle', '5': 'airplane',
            '6': 'bus', '7': 'train', '8': 'truck', '9': 'boat', '10': 'traffic light',
            '11': 'fire hydrant', '13': 'stop sign', '14': 'parking meter', '15': 'bench',
            '16': 'bird', '17': 'cat', '18': 'dog', '19': 'horse', '20': 'sheep',
            '21': 'cow', '22': 'elephant', '23': 'bear', '24': 'zebra', '25': 'giraffe',
            '27': 'backpack', '28': 'umbrella', '31': 'handbag', '32': 'tie', '33': 'suitcase',
            '34': 'frisbee', '35': 'skis', '36': 'snowboard', '37': 'sports ball', '38': 'kite',
            '39': 'baseball bat', '40': 'baseball glove', '41': 'skateboard', '42': 'surfboard',
            '43': 'tennis racket', '44': 'bottle', '46': 'wine glass', '47': 'cup', '48': 'fork',
            '49': 'knife', '50': 'spoon', '51': 'bowl', '52': 'banana', '53': 'apple',
            '54': 'sandwich', '55': 'orange', '56': 'broccoli', '57': 'carrot', '58': 'hot dog',
            '59': 'pizza', '60': 'donut', '61': 'cake', '62': 'chair', '63': 'couch',
            '64': 'potted plant', '65': 'bed', '67': 'dining table', '70': 'toilet', '72': 'tv',
            '73': 'laptop', '74': 'mouse', '75': 'remote', '76': 'keyboard', '77': 'cell phone',
            '78': 'microwave', '79': 'oven', '80': 'toaster', '81': 'sink', '82': 'refrigerator',
            '84': 'book', '85': 'clock', '86': 'vase', '87': 'scissors', '88': 'teddy bear',
            '89': 'hair drier', '90': 'toothbrush'
        }
        # Note: If your model outputs different IDs or names, you must change this dictionary!
        # Check your OAK-D ROS driver documentation or model files for the correct mapping.

        self.get_logger().info(f"Object ID-to-Name map loaded (using example COCO map).")


        # --- Define Object Name to Scene Map ---
        # Keys here should match the names from the ID_TO_NAME_MAP after lowercasing
        self.OBJECT_TO_SCENE_MAP = {
            "microwave": "nhà bếp",
            "oven": "nhà bếp",
            "refrigerator": "nhà bếp",
            "sink": "nhà bếp",
            "dining table": "nhà bếp", # Hoặc phòng ăn
            "tv": "phòng khách",
            "couch": "phòng khách",
            "sofa": "phòng khách", # Đồng nghĩa với couch
            "chair": "phòng khách", # Khá mơ hồ, có thể ở nhiều nơi
            "potted plant": "phòng khách", # Hoặc văn phòng, ban công
            "bed": "phòng ngủ",
            "laptop": "văn phòng",
            "computer": "văn phòng", # Thường dùng cho máy tính để bàn (need to map 'monitor', 'keyboard', 'mouse' perhaps)
            "keyboard": "văn phòng",
            "mouse": "văn phòng",
            "monitor": "văn phòng", # Add monitor if your ID map has it
            "book": "văn phòng", # Hoặc thư viện, phòng khách
            "toilet": "phòng tắm",
            "person": "không xác định rõ" # Người có thể ở bất kỳ đâu
            # Thêm các vật thể và môi trường khác nếu cần, đảm bảo tên khớp với ID_TO_NAME_MAP
        }
        self.get_logger().info(f"Object Name-to-Scene map loaded.")


        # --- Subscribers ---
        self.detection_subscriber = self.create_subscription(
            Detection3DArray,
            detection_topic,
            self.detection_callback,
            10)

        # Tùy chọn: Subscriber cho hình ảnh
        # self.image_subscriber = self.create_subscription(
        #     Image,
        #     image_topic,
        #     self.image_callback,
        #     10)
        # self.latest_image = None

        # --- Publisher ---
        self.scene_publisher = self.create_publisher(String, output_scene_topic, 10)

        self.last_inferred_scene = "không rõ"
        self.get_logger().info("Scene recognition Node started successfully.")

    # def image_callback(self, msg): # Tùy chọn
    #     self.latest_image = msg
    #     pass

    def detection_callback(self, msg: Detection3DArray):
        # self.get_logger().debug(f"Received {len(msg.detections)} detections.") # Bỏ comment nếu muốn xem debug log này
        detected_objects_this_frame = set()

        for detection in msg.detections: # msg.detections là một list các Detection2D
            if detection.results: # Kiểm tra xem có results không
                # detection.results là một list các ObjectHypothesisWithPose
                # Lấy ObjectHypothesisWithPose đầu tiên trong list results
                object_hypothesis_with_pose = detection.results[0] # Kiểu: vision_msgs/msg/ObjectHypothesisWithPose

                # Truy cập vào thuộc tính 'hypothesis' để lấy ObjectHypothesis
                hypothesis_data = object_hypothesis_with_pose.hypothesis # Kiểu: vision_msgs/msg/ObjectHypothesis

                # Lấy ID vật thể (dạng chuỗi) và score từ ObjectHypothesis
                object_id_str = hypothesis_data.class_id.strip() # Ví dụ: '11'
                score = hypothesis_data.score

                # --- Map ID to Name ---
                object_name = "unknown" # Tên mặc định nếu ID không tìm thấy trong map

                # Kiểm tra xem ID có trong dictionary ánh xạ không
                if object_id_str in self.ID_TO_NAME_MAP:
                     # Lấy tên vật thể tương ứng, chuyển sang chữ thường
                     object_name = self.ID_TO_NAME_MAP[object_id_str].lower()

                     # --- Process based on Name and Confidence ---
                     if score >= self.confidence_threshold:
                         # Nếu tên vật thể được tìm thấy và score đủ cao
                         self.get_logger().debug(f"Confident detection: '{object_name}' (ID: {object_id_str}) with score: {score:.2f}")
                         detected_objects_this_frame.add(object_name)
                     else:
                         # Nếu tên vật thể được tìm thấy nhưng score thấp
                         self.get_logger().debug(f"Detection '{object_name}' (ID: {object_id_str}) below threshold ({score:.2f} < {self.confidence_threshold}).")
                else:
                     # Nếu ID không có trong ID_TO_NAME_MAP
                     if score >= self.confidence_threshold:
                         # Nếu phát hiện ID lạ với score cao, có thể bạn muốn biết
                         self.get_logger().debug(f"Detected object with unknown ID: '{object_id_str}' with score: {score:.2f}. Ignoring for scene inference.")
                     # Bỏ qua các phát hiện có ID không xác định cho việc suy luận môi trường
                     continue # Bỏ qua phần xử lý tiếp theo cho phát hiện này

        # --- Logic suy luận môi trường dựa trên detected_objects_this_frame ---
        current_inferred_scene = "không rõ" # Mặc định
        highest_priority_scene = None # Lưu môi trường có độ ưu tiên cao nhất tìm được

        if not detected_objects_this_frame:
            self.get_logger().debug("No confident objects detected in this frame.")
            # Nếu không có vật thể nào tin cậy, môi trường vẫn là "không có vật thể"
            current_inferred_scene = "không có vật thể"
        else:
            self.get_logger().debug(f"Confident objects this frame: {detected_objects_this_frame}")
            # Lặp qua các tên vật thể đã được phát hiện và đủ tin cậy
            for obj_name in detected_objects_this_frame:
                # Kiểm tra xem tên vật thể có trong map ánh xạ sang môi trường không
                if obj_name in self.OBJECT_TO_SCENE_MAP:
                    potential_scene = self.OBJECT_TO_SCENE_MAP[obj_name]
                    # Logic ưu tiên đơn giản: tìm môi trường cụ thể (không phải "không xác định rõ")
                    if potential_scene != "không xác định rõ":
                        highest_priority_scene = potential_scene
                        break # Tìm thấy môi trường cụ thể, dừng tìm kiếm
                    elif highest_priority_scene is None:
                         # Nếu chưa tìm thấy môi trường cụ thể nào, nhưng tìm thấy "không xác định rõ"
                         # Gán tạm là "không xác định rõ" nhưng vẫn tiếp tục tìm kiếm môi trường cụ thể hơn
                         highest_priority_scene = potential_scene

            # Sau khi lặp qua tất cả các vật thể tin cậy:
            if highest_priority_scene:
                # Nếu tìm thấy ít nhất một môi trường (có thể là "không xác định rõ")
                current_inferred_scene = highest_priority_scene
            else:
                # Nếu có vật thể tin cậy nhưng không có vật thể nào trong OBJECT_TO_SCENE_MAP
                current_inferred_scene = "môi trường không xác định (có vật thể không rõ)" # Cập nhật lại thông báo này


        # --- Publish cảnh nếu thay đổi ---
        if current_inferred_scene != self.last_inferred_scene:
            scene_msg = String()
            scene_msg.data = current_inferred_scene
            self.scene_publisher.publish(scene_msg)
            self.last_inferred_scene = current_inferred_scene
            self.get_logger().info(f"Published new inferred scene: '{current_inferred_scene}'")
        else:
            self.get_logger().debug(f"Inferred scene '{current_inferred_scene}' is the same as last published. Not republishing.")


def main(args=None):
    rclpy.init(args=args)
    node = scene_recognition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

