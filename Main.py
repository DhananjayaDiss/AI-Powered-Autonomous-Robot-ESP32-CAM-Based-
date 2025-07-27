import asyncio
import websockets
import json
import cv2
import numpy as np
import base64
from datetime import datetime
import threading
import queue
from pathlib import Path

# AI/ML Libraries
import tensorflow as tf
from ultralytics import YOLO  # For object detection
import mediapipe as mp  # For pose detection

class RobotAI:
    def __init__(self, esp32_ip="192.168.1.100"):
        self.esp32_ip = esp32_ip
        self.esp32_port = 81
        self.websocket = None
        
        # AI Models
        self.yolo_model = YOLO('yolov8n.pt')  # Lightweight YOLO model
        self.pose_detector = mp.solutions.pose.Pose()
        
        # Data queues
        self.sensor_queue = queue.Queue()
        self.frame_queue = queue.Queue()
        
        # Robot state
        self.current_state = "IDLE"
        self.last_sensor_data = None
        self.obstacles_detected = []
        
        # Navigation parameters
        self.safe_distance = 30.0  # cm
        self.turn_duration = 1000  # ms
        self.forward_speed = 150
        self.turn_speed = 120
        
        print("Robot AI System Initialized")
    
    async def connect_to_esp32(self):
        """Connect to ESP32-CAM via WebSocket"""
        uri = f"ws://{self.esp32_ip}:{self.esp32_port}"
        try:
            self.websocket = await websockets.connect(uri)
            print(f"Connected to ESP32-CAM at {uri}")
            return True
        except Exception as e:
            print(f"Failed to connect to ESP32-CAM: {e}")
            return False
    
    async def listen_to_esp32(self):
        """Listen for data from ESP32-CAM"""
        try:
            async for message in self.websocket:
                data = json.loads(message)
                
                if data["type"] == "sensor_data":
                    self.sensor_queue.put(data)
                elif data["type"] == "camera_frame":
                    self.process_camera_frame(data)
                    
        except websockets.exceptions.ConnectionClosed:
            print("Connection to ESP32-CAM lost")
    
    def process_camera_frame(self, frame_data):
        """Process incoming camera frame"""
        try:
            # Decode base64 image
            image_data = base64.b64decode(frame_data["data"])
            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                self.frame_queue.put(frame)
        except Exception as e:
            print(f"Error processing camera frame: {e}")
    
    def analyze_frame_with_ai(self, frame):
        """Perform AI analysis on camera frame"""
        results = {}
        
        # Object Detection with YOLO
        yolo_results = self.yolo_model(frame)
        detected_objects = []
        
        for result in yolo_results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    if confidence > 0.5:  # Confidence threshold
                        class_name = self.yolo_model.names[class_id]
                        bbox = box.xyxy[0].tolist()
                        detected_objects.append({
                            'class': class_name,
                            'confidence': confidence,
                            'bbox': bbox
                        })
        
        results['objects'] = detected_objects
        
        # Edge Detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        
        # Analyze bottom portion for cliff/edge detection
        height, width = edges.shape
        bottom_region = edges[int(height*0.7):height, :]
        edge_pixels = np.sum(bottom_region > 0)
        edge_density = edge_pixels / (bottom_region.shape[0] * bottom_region.shape[1])
        
        results['edge_detected'] = edge_density > 0.1  # Threshold for edge detection
        results['edge_density'] = edge_density
        
        # Lane/Path Detection
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, 
                               minLineLength=50, maxLineGap=10)
        
        path_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Filter for roughly vertical lines (path edges)
                angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
                if abs(angle) > 60:  # Vertical-ish lines
                    path_lines.append(line[0])
        
        results['path_lines'] = path_lines
        results['path_detected'] = len(path_lines) >= 2
        
        return results
    
    async def send_command_to_esp32(self, command):
        """Send command to ESP32-CAM"""
        if self.websocket:
            try:
                await self.websocket.send(json.dumps(command))
            except Exception as e:
                print(f"Error sending command: {e}")
    
    async def move_robot(self, left_speed, right_speed, duration=0):
        """Send movement command to robot"""
        command = {
            "command": "move",
            "left_speed": left_speed,
            "right_speed": right_speed,
            "duration": duration
        }
        await self.send_command_to_esp32(command)
    
    async def get_camera_frame(self):
        """Request camera frame from ESP32"""
        command = {"command": "get_frame"}
        await self.send_command_to_esp32(command)
    
    async def play_sound(self, frequency=1000, duration=500):
        """Play sound on robot"""
        command = {
            "command": "beep",
            "frequency": frequency,
            "duration": duration
        }
        await self.send_command_to_esp32(command)
    
    def decision_making_algorithm(self, sensor_data, vision_results):
        """Main AI decision making logic"""
        decisions = {
            'action': 'STOP',
            'left_speed': 0,
            'right_speed': 0,
            'duration': 0,
            'reason': 'No decision'
        }
        
        # Priority 1: Edge/Cliff Detection
        if vision_results.get('edge_detected', False):
            decisions.update({
                'action': 'AVOID_EDGE',
                'left_speed': -self.turn_speed,
                'right_speed': self.turn_speed,
                'duration': self.turn_duration,
                'reason': 'Edge detected - avoiding cliff'
            })
            return decisions
        
        # Priority 2: Obstacle Avoidance
        distance = sensor_data.get('distance', 0)
        if distance > 0 and distance < self.safe_distance:
            # Obstacle detected - decide turn direction
            # Use IMU or additional sensors to determine best direction
            decisions.update({
                'action': 'AVOID_OBSTACLE',
                'left_speed': -self.turn_speed,
                'right_speed': self.turn_speed,
                'duration': self.turn_duration,
                'reason': f'Obstacle at {distance}cm - turning right'
            })
            return decisions
        
        # Priority 3: Object Following (e.g., follow person)
        objects = vision_results.get('objects', [])
        person_detected = any(obj['class'] == 'person' for obj in objects)
        
        if person_detected:
            person = next(obj for obj in objects if obj['class'] == 'person')
            bbox = person['bbox']
            frame_center = 320  # Assuming VGA width
            object_center = (bbox[0] + bbox[2]) / 2
            
            # Simple proportional control to center on person
            error = object_center - frame_center
            turn_adjustment = int(error * 0.5)  # Proportional gain
            
            decisions.update({
                'action': 'FOLLOW_PERSON',
                'left_speed': self.forward_speed - turn_adjustment,
                'right_speed': self.forward_speed + turn_adjustment,
                'duration': 500,
                'reason': f'Following person (error: {error:.1f})'
            })
            return decisions
        
        # Priority 4: Path Following
        if vision_results.get('path_detected', False):
            path_lines = vision_results['path_lines']
            # Calculate path center and adjust movement
            # Simplified path following logic
            decisions.update({
                'action': 'FOLLOW_PATH',
                'left_speed': self.forward_speed,
                'right_speed': self.forward_speed,
                'duration': 500,
                'reason': 'Following detected path'
            })
            return decisions
        
        # Default: Explore/Move Forward
        if distance == 0 or distance > self.safe_distance:
            decisions.update({
                'action': 'EXPLORE',
                'left_speed': self.forward_speed,
                'right_speed': self.forward_speed,
                'duration': 1000,
                'reason': 'Exploring - path clear'
            })
        
        return decisions
    
    async def main_control_loop(self):
        """Main control loop with AI processing"""
        print("Starting main control loop...")
        
        while True:
            try:
                # Get camera frame
                await self.get_camera_frame()
                
                # Process sensor data
                if not self.sensor_queue.empty():
                    sensor_data = self.sensor_queue.get()
                    self.last_sensor_data = sensor_data
                
                # Process camera frame
                vision_results = {}
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    vision_results = self.analyze_frame_with_ai(frame)
                    
                    # Optional: Save annotated frame for debugging
                    self.save_debug_frame(frame, vision_results)
                
                # Make decisions based on AI analysis
                if self.last_sensor_data:
                    decisions = self.decision_making_algorithm(
                        self.last_sensor_data, vision_results
                    )
                    
                    print(f"Decision: {decisions['action']} - {decisions['reason']}")
                    
                    # Execute decision
                    await self.move_robot(
                        decisions['left_speed'],
                        decisions['right_speed'],
                        decisions['duration']
                    )
                    
                    # Audio feedback
                    if decisions['action'] in ['AVOID_EDGE', 'AVOID_OBSTACLE']:
                        await self.play_sound(2000, 300)  # Warning beep
                    elif decisions['action'] == 'FOLLOW_PERSON':
                        await self.play_sound(1000, 200)  # Following beep
                
                # Control loop frequency
                await asyncio.sleep(0.2)  # 5 Hz control loop
                
            except Exception as e:
                print(f"Error in control loop: {e}")
                await asyncio.sleep(1)
    
    def save_debug_frame(self, frame, vision_results):
        """Save annotated frame for debugging"""
        debug_frame = frame.copy()
        
        # Draw detected objects
        for obj in vision_results.get('objects', []):
            bbox = obj['bbox']
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{obj['class']}: {obj['confidence']:.2f}"
            cv2.putText(debug_frame, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw path lines
        for line in vision_results.get('path_lines', []):
            x1, y1, x2, y2 = line
            cv2.line(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        
        # Add status text
        status_text = f"Edge: {vision_results.get('edge_detected', False)}"
        cv2.putText(debug_frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Save frame (optional - for debugging)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        cv2.imwrite(f"debug_frames/frame_{timestamp}.jpg", debug_frame)
    
    async def run(self):
        """Main run function"""
        # Connect to ESP32-CAM
        if not await self.connect_to_esp32():
            return
        
        # Start listening and control tasks
        listen_task = asyncio.create_task(self.listen_to_esp32())
        control_task = asyncio.create_task(self.main_control_loop())
        
        # Run both tasks concurrently
        await asyncio.gather(listen_task, control_task)

# Usage
if __name__ == "__main__":
    # Create robot AI instance
    robot = RobotAI(esp32_ip="192.168.1.100")  # Replace with your ESP32-CAM IP
    
    # Run the system
    asyncio.run(robot.run())