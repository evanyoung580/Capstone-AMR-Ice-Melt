import cv2
import numpy as np
import yaml
import zmq
import json

with open('/home/amrmgr/amr/config/opencv_config.yaml', 'r') as config_file:
    config = yaml.safe_load(config_file)

    if config['image_proc']['debug']:
        detect = True
    else:
        detect = False

vision_context = zmq.Context()
vision_socket = vision_context.socket(zmq.PAIR)
vision_socket.connect("ipc:///home/amrmgr/amr/tmp/vision_socket")

cap = cv2.VideoCapture(config['webcam']['cam_index'])
if not cap.isOpened():
    print("Error: Unable to access the webcam.")
    exit()
while True:
    if detect == True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Error: Unable to read frame from the webcam.")
            break

        # Crop frame
        height, width, _ = frame.shape
        top_crop = int(height * config['image_proc']['crop']['top'])
        bottom_crop = int(height * config['image_proc']['crop']['bottom'])
        cropped_frame = frame[top_crop:bottom_crop, :]

        # resize frame
        k_rescale = config['image_proc']['rescale']
        height, width, _ = cropped_frame.shape
        resized_frame = cv2.resize(cropped_frame, (width * k_rescale, height * k_rescale))
        height, width, _ = resized_frame.shape


        # Convert the cropped frame to HSV
        hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

        # BGR to HSV and max and min values
        sidewalk_bgr = config['image_proc']['color_detect']['bgr_targ']
        sidewalk_bgr = np.uint8([[sidewalk_bgr]])
        sidewalk_hsv = cv2.cvtColor(sidewalk_bgr, cv2.COLOR_BGR2HSV)
        tolerance = config['image_proc']['color_detect']['hue_tol']
        hsv_min = sidewalk_hsv[0][0][0] - tolerance, \
            config['image_proc']['color_detect']['sat']['min'], \
            config['image_proc']['color_detect']['val']['min']
        hsv_max = sidewalk_hsv[0][0][0] + tolerance, \
            config['image_proc']['color_detect']['sat']['max'], \
            config['image_proc']['color_detect']['val']['max']
        hsv_min = np.array(hsv_min, dtype=np.uint8)
        hsv_max = np.array(hsv_max, dtype=np.uint8)

        # mask in range color values
        mask = cv2.inRange(hsv_frame, hsv_min, hsv_max)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            sidewalk_pos = [x + (w // 2), y + (h // 2)]
            sidewalk_targ = [((sidewalk_pos[0] - (x / 2)) / 2), ((sidewalk_pos[1] - (y / 2)) / 2)]
            if config['image_proc']['debug']:
                cv2.rectangle(resized_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(resized_frame, sidewalk_pos, 5, (0, 255, 0), -1)
            out_message = {"status": "detecting", "sw_detected": "True", "sw_pos": {"x": sidewalk_targ[0], "y": sidewalk_targ[1]}}
        else:
            out_message = {"status": "detecting", "sw_detected": "False"}
        vision_socket.send_json(out_message)
        
        if config['image_proc']['debug']:
            cv2.imshow("original", frame)
            cv2.imshow("cropped", resized_frame)
            cv2.imshow("hsv", hsv_frame)
            cv2.imshow("mask", mask)
        if cv2.waitKey(1000//config['webcam']['fps']) & 0xFF == ord('q'):
            break
    if vision_socket.poll(timeout=10):
        in_message = vision_socket.recv_json(flags=zmq.NOBLOCK)
        if (in_message['manage']['detect'] == True):
            detect = True
        elif (in_message['manage']['detect'] == False):
            detect = False
            out_message = {"status": "idle"}
            vision_socket.send_json(out_message)
    
out_message = {"status": "stopped"}
vision_socket.send_json(out_message)
cap.release()
cv2.destroyAllWindows()
vision_socket.close()
vision_context.term()