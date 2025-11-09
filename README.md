# Project Description
In this project, we combine YOLO detection on edge device, with TurtleBot3 motion controls such as lane detection, PID control and traffic sign related actions.
# YOLOxTB3
This system integrates a YOLOv8n detection model and PD motion control on a Raspberry Pi 4. The model identifies 4 types of traffic signs in real time, with different LED colors indicating the results due to limited processing resources on the Raspberry Pi. 

## Motivation
As modern society relies more on intelligent systems like autonomous vehicles and smart cities, edge computing enables real-time processing and reliable decision-making by reducing latency, instability, and network dependence. Therefore, this project adopts an on-edge design that allows the Raspberry Pi 4 to handle all computations locally. Building on our previous Scale-Invariant Feature Transform (SIFT) algorithm, which had slower detection and limited adaptability, we now use You-Only-Look-Once (YOLO) for real-time, accurate, and flexible traffic sign detection. 

## Methodology
### Optimization for Edge Device
- YOLOv8n: best suited to the Coral TPU & Raspberry Pi environment  
To improve sign detection accuracy under varied lighting and angles, dataset augmentation (e.g., rotation, scaling, and mosaic transformations) is applied.  
Precision=0.994, Recall=1, mAP50=0.995, mAP50-95=0.849 
- Docker:  
To address environment inconsistencies and simplify deployment on the Raspberry Pi 4, the system uses Docker containers to isolate each module along with its required dependencies.  
To enable real-time data exchange (e.g., detection results) and parameter updates between isolated containers, Docker Compose connects them through a bridge network.
- Coral TPU:  
To overcome the limited processing power of the Raspberry Pi, YOLO inference is offloaded to the TPU, cutting detection latency by half (from 8.6 s to 4.3 s).

