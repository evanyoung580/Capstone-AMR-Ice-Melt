import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

def object_detection(frame):
    # annotated_frame = frame.copy()
    # Load the pre-trained YOLOv8 model
   

    # List of labels considered obstacles (can adjust based on your environment)
    obstacle_classes = ['car', 'truck', 'deer', 'dog', 'birds', 'cell phone']
    
    results = model(frame, verbose = False)[0]

    for r in results.boxes:
        cls_id = int(r.cls[0]) #Class index
        label = model.names[cls_id] #Class name
        conf = float(r.conf[0]) #Confidence Score 

        if conf < 0.5:
            continue  # Skip low confidence
        #Coordinate of the bounding box
        x1, y1, x2, y2 = map(int, r.xyxy[0])
        if label == 'person':
            color = (0, 255, 0)
        else:
            color = (0, 0, 255) 
            
        #Draw a box if it is a person or within the obstcle classes 
        if label == 'person' or label in obstacle_classes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        #cv2.imshow("Object Detection", frame)
        #cv2.imshow("Object Detection", annotated_frame)
        return frame
