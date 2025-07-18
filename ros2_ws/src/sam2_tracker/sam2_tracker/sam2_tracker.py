#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PolygonInstance, PolygonInstanceStamped, Point32
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from sam2.build_sam import build_sam2_camera_predictor

class SAMTracker(Node):
    def __init__(self):
        super().__init__('sam_tracker')
        
        self.bridge = CvBridge()
        self.declare_parameter('output_quality', 75)
        
        input_topic = '/camera/image/compressed'
        output_topic = '/masks/compressed'
        self.output_quality = self.get_parameter('output_quality').value
        
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            CompressedImage,
            output_topic,
            10)
        
        self.poly_pub = self.create_publisher(
            PolygonInstanceStamped,
            'polygon',
            10)
        
        sam2_checkpoint = "/home/user/segment-anything-2/checkpoints/sam2.1_hiera_large.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"

        self.predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)
        self.if_init = False

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)            
            processed_image, polygons = self.process_image(cv_image)            
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(
                processed_image,
                dst_format='jpg')  # or 'png' if you prefer lossless            
            compressed_msg.header = msg.header
            self.publisher.publish(compressed_msg)
            for poly in polygons:
                polygon_msg = PolygonInstanceStamped()
                polygon_msg.header = msg.header
                polygon_msg.polygon = poly
                self.poly_pub.publish(polygon_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def process_image(self, image):
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        width, height = frame.shape[:2][::-1]
        polygons = []
        if not self.if_init:
            self.predictor.load_first_frame(frame)
            self.if_init = True
            ann_frame_idx = 0
            ann_obj_id = 1
            points = np.array([[595, 65], [480,150], [472,120], [425,170], [395,188]], dtype=np.float32)
            labels = np.array([1,0,0,1,1], dtype=np.int32)
            _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                frame_idx=ann_frame_idx, obj_id=ann_obj_id, points=points, labels=labels,
            )
            ann_obj_id = 2
            points = np.array([[350, 275]], dtype=np.float32)
            labels = np.array([1], np.int32)
            _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                frame_idx=ann_frame_idx, obj_id=ann_obj_id, points=points, labels=labels, clear_old_points=False
            )
            points = np.array([[298, 210]], dtype=np.float32)
            labels = np.array([1], np.int32)
            ann_obj_id = 3
            _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                frame_idx=ann_frame_idx, obj_id=ann_obj_id, points=points, labels=labels, clear_old_points=False
            )
            processed_image = np.zeros((height, width, 3), dtype=np.uint8)
        else:
            out_obj_ids, out_mask_logits = self.predictor.track(frame)
            all_mask = np.zeros((height, width, 3), dtype=np.uint8)
            all_mask[..., 1] = 255
            for i in range(0, len(out_obj_ids)):
                out_mask = (out_mask_logits[i] > 0.0).permute(1, 2, 0).cpu().numpy().astype(
                    np.uint8
                ) * 255
                contours, _ = cv2.findContours(out_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    poly = PolygonInstance()
                    poly.id = out_obj_ids[i]
                    all_points = np.vstack([contour.squeeze() for contour in contours])
                    hull = cv2.convexHull(all_points)
                    for point in hull.squeeze():
                        p = Point32()
                        p.x = float(point[0])
                        p.y = float(point[1])
                        p.z = 0.0
                        poly.polygon.points.append(p)
                    polygons.append(poly)
    
                hue = (i + 3) / (len(out_obj_ids) + 3) * 255
                all_mask[out_mask[..., 0] == 255, 0] = hue
                all_mask[out_mask[..., 0] == 255, 2] = 255
            processed_image = cv2.cvtColor(all_mask, cv2.COLOR_HSV2RGB)
        return processed_image, polygons

def main(args=None):
    torch.autocast(device_type="cuda", dtype=torch.bfloat16).__enter__()

    if torch.cuda.get_device_properties(0).major >= 8:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
    
    rclpy.init(args=args)
    sam_tracker = SAMTracker()
    rclpy.spin(sam_tracker)
    sam_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

