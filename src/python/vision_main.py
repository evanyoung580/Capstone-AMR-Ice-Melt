import cv2
import numpy as np
import yaml
import zmq
import time
from Integrated_Object_Detection import object_detection


class VisionProcessor:
    def __init__(self, config_path, telem_socket_path, cmd_socket_path):
        # Load configuration
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        # Initialize ZMQ communication
        self.telem_context = zmq.Context()
        self.cmd_context = zmq.Context()
        self.telem_socket = self.telem_context.socket(zmq.PUSH)
        self.cmd_socket = self.cmd_context.socket(zmq.PULL)
        self.telem_socket.bind(telem_socket_path)
        self.cmd_socket.connect(cmd_socket_path)
        
        # Initialize webcam
        webcam_index = self.config['webcam']['cam_index']
        self.cap = cv2.VideoCapture(webcam_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Error: Unable to access the webcam at index {webcam_index}.")
        
        self.detect = self.config['image_proc']['debug']

    def process_color(self, frame, out_frame):
        # Convert to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for color detection
        sidewalk_bgr = np.uint8([[self.config['image_proc']['clr_detect']['bgr_targ']]])
        sidewalk_hsv = cv2.cvtColor(sidewalk_bgr, cv2.COLOR_BGR2HSV)
        tolerance = self.config['image_proc']['clr_detect']['hue_tol']
        hsv_min = np.array([
            sidewalk_hsv[0][0][0] - tolerance,
            self.config['image_proc']['clr_detect']['sat']['min'],
            self.config['image_proc']['clr_detect']['val']['min']
        ], dtype=np.uint8)
        hsv_max = np.array([
            sidewalk_hsv[0][0][0] + tolerance,
            self.config['image_proc']['clr_detect']['sat']['max'],
            self.config['image_proc']['clr_detect']['val']['max']
        ], dtype=np.uint8)

        # Mask based on HSV range
        mask = cv2.inRange(hsv_frame, hsv_min, hsv_max)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        telem = {"sw_detected": "False"}
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            sidealk_area = cv2.contourArea(largest_contour)
            if sidealk_area > self.config['image_proc']['clr_detect']['min_area']:
                fh, fw, _ = frame.shape
                x, y, w, h = cv2.boundingRect(largest_contour)
                left_targ = [x / fw, (y + (h // 2)) / fh]
                right_targ = [(x + w) / fw, (y + (h // 2)) / fh]
                center_targ = [(x + (w // 2)) / fw, (y + (h // 2)) / fh]

                if self.config['image_proc']['debug']:
                    cv2.rectangle(out_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(out_frame, (x + (w // 2), y + (h // 2)), 5, (0, 255, 0), -1)

                telem = {
                    "sw_detected": "True",
                    "sw_pos": 
                    {
                        "left": {"x": left_targ[0], "y": left_targ[1]},
                        "center": {"x": center_targ[0], "y": center_targ[1]},
                        "right": {"x": right_targ[0], "y": right_targ[1]},
                        "area": sidealk_area
                    }
                }
        return telem, out_frame, mask
    
    def make_line_length(self, longest):
        def line_length(line):
            x1, y1, x2, y2 = line[0] 
            angle = np.arctan2(y2 - y1, x2 - x1) * (180 / np.pi)
            ang_excl = self.config['image_proc']['line_detect']['line']['ang_excl']
            if ((angle > ang_excl) and (angle < 180 - ang_excl)) or \
                ((angle < -ang_excl) and (angle > -(180 - ang_excl))):
                if longest is not None:
                    x1ll, _, x2ll, _ = longest[0]
                    x_gap = self.config['image_proc']['line_detect']['line']['x_gap']
                    x22d = abs(x2-x2ll) > x_gap
                    x11d = abs(x1-x1ll) > x_gap
                    x21d = abs(x2-x1ll) > x_gap
                    x12d = abs(x1-x2ll) > x_gap
                    if (x22d and x11d and x21d and x12d):
                        length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                    else:
                        length = 0
                else:
                    length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            else: 
                length = 0
            return length
        return line_length

    def process_line(self, frame, out_frame):
        height, width, _ = frame.shape
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur_matrix = self.config['image_proc']['line_detect']['canny']['blur']
        blur_frame = cv2.blur(gray_frame, (blur_matrix, blur_matrix))
        edge_frame = cv2.Canny(blur_frame, self.config['image_proc']['line_detect']['canny']['low'], 
                            self.config['image_proc']['line_detect']['canny']['low'])
        lines = cv2.HoughLinesP(edge_frame, 1, np.pi / 180,
                                self.config['image_proc']['line_detect']['hough']['thresh'], 
                                minLineLength=self.config['image_proc']['line_detect']['hough']['min_len'], 
                                maxLineGap=self.config['image_proc']['line_detect']['hough']['max_gap'])
        telem = {"sw_detected": "False"}
        if lines is not None:
            longest = self.make_line_length(None)
            long_line = max(lines, key=longest)
            sec_longest = self.make_line_length(long_line)
            sec_long_line = max(lines, key=sec_longest)
            long_line_len = longest(long_line)
            sec_long_line_len = longest(sec_long_line)
            if (long_line_len > 0) and (sec_long_line_len > 0):
                x1ll, y1ll, x2ll, y2ll = long_line[0]
                x1sl, y1sl, x2sl, y2sl = sec_long_line[0]
                if x1ll < x1sl:
                    left_line = long_line
                    right_line = sec_long_line
                    x1ll, y1ll, x2ll, y2ll = left_line[0]
                    x1rl, y1rl, x2rl, y2rl = right_line[0]
                else:
                    left_line = sec_long_line
                    right_line = long_line
                    x1ll, y1ll, x2ll, y2ll = left_line[0]
                    x1rl, y1rl, x2rl, y2rl = right_line[0]

                left_targ = [((np.average([x1ll, x2ll]) / width)) - 1, ((np.average([y1ll, y2ll])) / height)]
                right_targ = [((np.average([x1rl, x2rl]) / width)) - 1, ((np.average([y1rl, y2rl])) / height)]
                center_targ = [np.average([left_targ[0], right_targ[0]]), np.average([left_targ[1], right_targ[1]])]

                if self.config['image_proc']['debug']:
                    cv2.line(out_frame, (x1ll, y1ll), (x2ll, y2ll), (0, 0, 255), 4)
                    cv2.line(out_frame, (x1rl, y1rl), (x2rl, y2rl), (255, 0, 0), 4)

                telem = {
                    "sw_detected": "True",
                    "sw_pos": 
                    {
                        "left": {"x": float(left_targ[0]), "y": float(left_targ[1])},
                        "center": {"x": float(center_targ[0]), "y": float(center_targ[1])},
                        "right": {"x": float(right_targ[0]), "y": float(right_targ[1])}
                    }
                }
        return telem, out_frame, edge_frame

    def process_frame(self, frame):
        """Processes the frame: cropping, resizing, color detection, and contour analysis."""
        # Crop frame
        height, width, _ = frame.shape
        top_crop = int(height * self.config['image_proc']['crop']['top'])
        bottom_crop = int(height * self.config['image_proc']['crop']['bottom'])
        cropped_frame = frame[top_crop:bottom_crop, :]

        # Resize frame
        c_height, c_width, _ = cropped_frame.shape
        k_rescale = self.config['image_proc']['rescale']
        resized_frame = cv2.resize(
            cropped_frame, 
            (int(c_width * k_rescale), int(c_height * k_rescale))
        )

        return resized_frame

    def send_message(self, message):
        """Sends a JSON message via the socket."""
        if self.config['image_proc']['debug']:
                print(message)
        try:
            self.telem_socket.send_json(message, flags=zmq.NOBLOCK)
        except zmq.Again as e:
            print(f"Warning: Message sending failed: {e}")

    def receive_message(self):
        """Receives a JSON message from the socket and updates detection status."""
        if self.cmd_socket.poll(timeout=10):
            in_message = self.cmd_socket.recv_json(flags=zmq.NOBLOCK)
            if in_message['manage']['state'] == "detect":
                self.detect = True
            elif in_message['manage']['state'] == "calibrate":
                self.detect = False
                self.send_message({"status": "calibrating"})
                self.calibrate()
            else: 
                self.detect = False
                self.send_message({"status": "idle"})

    def cleanup(self):
        """Releases resources."""
        self.cap.release()
        cv2.destroyAllWindows()
        self.telem_socket.close()
        self.cmd_socket.close()
        self.telem_context.term()
        self.cmd_context.term()
    

    def calibrate(self):
        """Calibrates new detection hue value with central camera color."""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            print("Error: Unable to read frame from the webcam.")
        
        resized_frame = self.process_frame(frame)
        height, width, _ = resized_frame.shape
        center_size = 50    
        x_start = width // 2 - center_size // 2
        x_end = width // 2 + center_size // 2
        y_start = height // 2 - center_size // 2
        y_end = height // 2 + center_size // 2

        center_frame = resized_frame[y_start:y_end, x_start:x_end]
        blur_cent_frame = cv2.blur(center_frame, (15, 15))
        avr_bgr = blur_cent_frame.mean(axis=(0, 1))

        avr_bgr_list = np.round(avr_bgr).astype(int).tolist()

        self.config['image_proc']['clr_detect']['bgr_targ'] = avr_bgr_list

        with open('/home/amrmgr/amr/config/opencv_config.yaml', 'w') as write_file:
            yaml.safe_dump(self.config, write_file, default_flow_style=False)
            
    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            if self.detect_scene_change(frame):
                print("Recalibrating.")
                self.calibrate()
                
    def run(self):
        """Main processing loop."""
        try:
            while True:
                if self.detect:
                    ret, raw_frame = self.cap.read()
                    if not ret or raw_frame is None:
                        print("Error: Unable to read frame from the webcam.")
                        break
                    
                    targets = object_detection(raw_frame)
                    cv2.imshow("20 points", targets)
            

                    frame = self.process_frame(raw_frame)
                    col_telem, out_frame, col_mask = self.process_color(frame)
                    line_telem, out_frame, line_mask = self.process_line(frame)

                    if self.config['image_proc']['debug']:
                        cv2.imshow("out_frame", out_frame)
                        cv2.imshow("col_mask", col_mask)
                        cv2.imshow("line_mask", line_mask)
                        cv2.waitKey(1000 // self.config['webcam']['fps'])

                    out_message = {
                        "status": "detecting",
                        "color_det": col_telem,
                        "line_det": line_telem
                    }
                else:
                    out_message = {"status": "idle"}
                self.send_message(out_message)
                #self.receive_message()
            self.send_message({"status": "stopped"})

        finally:
            self.cleanup()


if __name__ == "__main__":
    CONFIG_PATH = '/home/amrmgr/amr/config/opencv_config.yaml'
    TELEM_SOCKET_PATH = "ipc:///home/amrmgr/amr/tmp/vision_telem"
    CMD_SOCKET_PATH = "ipc:///home/amrmgr/amr/tmp/vision_cmd"

    vision_processor = VisionProcessor(CONFIG_PATH, TELEM_SOCKET_PATH, CMD_SOCKET_PATH)
    vision_processor.run()
