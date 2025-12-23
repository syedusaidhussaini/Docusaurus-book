# Chapter 11: Vision Perception and Scene Understanding

## Learning Objectives

After completing this chapter, you will be able to:
- Implement computer vision fundamentals for robotics applications
- Create object detection and recognition systems for robotic environments
- Understand scene understanding and spatial reasoning techniques
- Configure camera calibration and visual processing pipelines
- Integrate vision with language understanding for multimodal perception

## 11.1 Computer Vision Fundamentals for Robotics

Computer vision in robotics enables robots to perceive and understand their environment through visual sensors. For humanoid robots, this includes understanding objects, people, and spatial relationships in 3D space.

### Key Components of Robotic Vision:
- **Image Acquisition**: Capturing images from cameras mounted on the robot
- **Preprocessing**: Enhancing image quality and correcting sensor artifacts
- **Feature Extraction**: Identifying distinctive elements in images
- **Object Recognition**: Identifying and classifying objects in the scene
- **Scene Understanding**: Interpreting spatial relationships and context

### Vision Sensor Types for Humanoid Robots:
- **RGB Cameras**: Standard color cameras for object recognition
- **Depth Sensors**: RGB-D cameras or LiDAR for 3D understanding
- **Stereo Cameras**: Two cameras for depth estimation
- **Thermal Cameras**: For detecting heat signatures

## 11.2 Object Detection and Recognition in Robotic Environments

Object detection and recognition form the core of visual perception for humanoid robots:

### Popular Object Detection Models:
- **YOLO (You Only Look Once)**: Real-time object detection with good speed/accuracy trade-off
- **Faster R-CNN**: High accuracy for complex scenes
- **DETR (DEtection TRansformer)**: Transformer-based approach with strong performance

### Example Object Detection Implementation:
```python
import cv2
import torch
import torchvision.transforms as T
from torchvision.models.detection import fasterrcnn_resnet50_fpn

# Load pre-trained model
model = fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()

# Define COCO class names
COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
    'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
    'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
    'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
    'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

def detect_objects(image):
    # Preprocess image
    transform = T.Compose([T.ToTensor()])
    img_tensor = transform(image).unsqueeze(0)

    # Perform detection
    with torch.no_grad():
        predictions = model(img_tensor)

    # Extract results
    boxes = predictions[0]['boxes'].cpu().numpy()
    labels = predictions[0]['labels'].cpu().numpy()
    scores = predictions[0]['scores'].cpu().numpy()

    # Filter results by confidence threshold
    threshold = 0.5
    filtered_results = []
    for i in range(len(boxes)):
        if scores[i] > threshold:
            filtered_results.append({
                'box': boxes[i],
                'label': COCO_INSTANCE_CATEGORY_NAMES[labels[i]],
                'score': scores[i]
            })

    return filtered_results
```

## 11.3 Scene Understanding and Spatial Reasoning

Scene understanding goes beyond object detection to interpret the spatial relationships and context:

### Spatial Reasoning Components:
- **3D Reconstruction**: Building 3D models from 2D images
- **Pose Estimation**: Determining the position and orientation of objects
- **Spatial Relationships**: Understanding how objects relate to each other
- **Semantic Segmentation**: Classifying each pixel in the image

### Example Scene Understanding Pipeline:
```python
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class SceneUnderstanding:
    def __init__(self):
        self.object_detector = self.load_object_detector()
        self.depth_estimator = self.load_depth_estimator()
        self.pose_estimator = self.load_pose_estimator()

    def analyze_scene(self, rgb_image, depth_image):
        # Detect objects in the scene
        objects = self.detect_objects(rgb_image)

        # Estimate depth for each object
        scene_data = []
        for obj in objects:
            bbox = obj['box']
            depth_region = depth_image[int(bbox[1]):int(bbox[3]), int(bbox[0]):int(bbox[2])]
            avg_depth = np.mean(depth_region[depth_region > 0])  # Filter out invalid depth values

            # Estimate 3D position
            center_x = (bbox[0] + bbox[2]) / 2
            center_y = (bbox[1] + bbox[3]) / 2
            obj_3d_pos = self.pixel_to_3d(center_x, center_y, avg_depth)

            scene_data.append({
                'label': obj['label'],
                'bbox': bbox,
                'position_3d': obj_3d_pos,
                'confidence': obj['score']
            })

        return scene_data

    def pixel_to_3d(self, x, y, depth):
        # Convert pixel coordinates to 3D world coordinates
        # This requires camera intrinsic parameters
        fx, fy = 525.0, 525.0  # Focal lengths
        cx, cy = 319.5, 239.5  # Principal point

        # Convert to camera coordinates
        x_cam = (x - cx) * depth / fx
        y_cam = (y - cy) * depth / fy

        # Return 3D position relative to camera
        return np.array([x_cam, y_cam, depth])
```

## 11.4 Camera Calibration and Visual Processing Pipelines

Proper camera calibration is essential for accurate depth estimation and spatial reasoning:

### Calibration Process:
- **Intrinsic Parameters**: Focal length, principal point, distortion coefficients
- **Extrinsic Parameters**: Position and orientation of camera relative to robot
- **Calibration Patterns**: Checkerboard patterns for accurate parameter estimation

### Example Calibration Implementation:
```python
import cv2
import numpy as np

def calibrate_camera(images, pattern_size=(9, 6)):
    # Prepare object points
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return ret, mtx, dist, rvecs, tvecs

# Usage example
def process_camera_input(image, camera_matrix, dist_coeffs):
    # Undistort image
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )

    dst = cv2.undistort(image, camera_matrix, dist_coeffs, None, newcameramtx)

    # Crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    return dst
```

## 11.5 Integration of Vision with Language Understanding

Multimodal perception combines visual and linguistic information:

### Vision-Language Integration Approaches:
- **Grounded Language Understanding**: Linking language to visual elements
- **Visual Question Answering**: Answering questions about visual scenes
- **Referring Expression Comprehension**: Identifying objects based on language descriptions
- **Multimodal Embeddings**: Joint representation of visual and linguistic features

### Example Vision-Language Integration:
```python
import clip
import torch

class VisionLanguageIntegrator:
    def __init__(self):
        # Load CLIP model for vision-language understanding
        self.model, self.preprocess = clip.load("ViT-B/32")
        self.model.eval()

    def find_object_by_description(self, image, description):
        # Preprocess image
        image_input = self.preprocess(image).unsqueeze(0)

        # Tokenize text
        text_input = clip.tokenize([description])

        # Get similarity scores
        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_input)

            # Compute similarity
            similarity = torch.cosine_similarity(image_features, text_features)

        return similarity.item()

    def classify_scene_objects(self, image, object_descriptions):
        # Preprocess image
        image_input = self.preprocess(image).unsqueeze(0)

        # Tokenize all object descriptions
        text_inputs = clip.tokenize(object_descriptions)

        # Get similarity scores for all descriptions
        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_inputs)

            # Compute similarities
            similarities = torch.cosine_similarity(
                image_features.unsqueeze(1),
                text_features.unsqueeze(0),
                dim=2
            )

        # Get best matches for each region
        return similarities.squeeze(0).tolist()
```

## 11.6 Real-time Visual Processing Considerations

For humanoid robots, real-time performance is critical:

### Optimization Strategies:
- **Model Compression**: Use quantized or pruned models for faster inference
- **Multi-threading**: Separate image capture, processing, and action execution
- **Region of Interest**: Focus processing on relevant parts of the image
- **Temporal Consistency**: Use previous frames to reduce redundant computation

## Exercises

1. Implement object detection on a sample image using a pre-trained model
2. Create a camera calibration pipeline using checkerboard patterns
3. Develop a vision-language integration system using CLIP
4. Build a real-time processing pipeline with optimized inference

## Summary

This chapter covered computer vision fundamentals for robotics, including object detection, scene understanding, camera calibration, and vision-language integration. You learned how to create visual perception systems that enable humanoid robots to understand their environment and interact with objects. These capabilities are essential for the Vision → Language → Action pipeline in autonomous humanoid behavior.