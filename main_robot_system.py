# main_robot_system.py
import asyncio
from robot_ai import RobotAI
from object_tracker import ObjectTracker
from slam import SimpleSLAM

class AdvancedRobotSystem(RobotAI):
    def __init__(self, esp32_ip):
        super().__init__(esp32_ip)
        self.tracker = ObjectTracker()
        self.slam = SimpleSLAM()
        
    def advanced_decision_making(self, sensor_data, vision_results):
        """Enhanced decision making with tracking and SLAM"""
        # Use tracked objects for more stable decisions
        tracked_objects = self.tracker.get_tracked_objects()
        
        # Update SLAM
        self.slam.update_pose(sensor_data, vision_results)
        
        # Make decisions based on enhanced data
        return super().decision_making_algorithm(sensor_data, vision_results)

if __name__ == "__main__":
    robot = AdvancedRobotSystem("192.168.1.100")
    asyncio.run(robot.run())