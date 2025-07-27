# AI-Powered-Autonomous-Robot-ESP32-CAM-Based-
AI robot that integrates real-time video processing, obstacle avoidance, edge tracking, and voice feedback. The system leverages multi-sensor fusion and modular architecture to enable autonomous mobility and interaction.
System Architecture Overview
This project uses a hybrid architecture where:
┌─────────────────┐    WiFi/Serial    ┌──────────────────────┐
│   ESP32-CAM     │ ◄─────────────► │   Python System      │
│                 │                  │   (Raspberry Pi/PC)  │
│ • Camera        │                  │ • OpenCV             │
│ • Sensors       │                  │ • TensorFlow/PyTorch │
│ • Motors        │                  │ • AI Processing      │
│ • Real-time I/O │                  │ • Path Planning      │
└─────────────────┘                  └──────────────────────┘