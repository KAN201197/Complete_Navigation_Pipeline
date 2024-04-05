#!/usr/bin/env python3
import os
from ultralytics import YOLO


current_dir = os.path.dirname(os.path.abspath(__file__))
weights_path = os.path.join(current_dir, '..', 'yolov8', 'best.pt')
model = YOLO(weights_path)

