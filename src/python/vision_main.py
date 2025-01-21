import cv2
import numpy as np
import yaml
import zmq
import time


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

        # Convert to HSV
        hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

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

        return resized_frame, hsv_frame, mask, contours

    def analyze_contours(self, contours, frame):
        """Analyzes contours to detect and mark the sidewalk."""
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            sidewalk_pos = [x + (w // 2), y + (h // 2)]
            fh, fw, _ = frame.shape
            sidewalk_targ = [(2 * (sidewalk_pos[0] / fw)) - 1, ((fh - sidewalk_pos[1]) / fh)]

            if self.config['image_proc']['debug']:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, tuple(sidewalk_pos), 5, (0, 255, 0), -1)

            return {
                "status": "detecting",
                "sw_detected": "True",
                "sw_pos": {"x": sidewalk_targ[0], "y": sidewalk_targ[1]}
            }
        else:
            return {"status": "detecting", "sw_detected": "False"}

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
        ret, frame = self.cap.read()
        if not ret or frame is None:
            print("Error: Unable to read frame from the webcam.")
        
        resized_frame, _, _, _ = self.process_frame(frame)
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
            yaml.safe_dump(self.config, write_file, default_flow_style=True)

    def run(self):
        """Main processing loop."""
        try:
            while True:
                if self.detect:
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        print("Error: Unable to read frame from the webcam.")
                        break

                    resized_frame, hsv_frame, mask, contours = self.process_frame(frame)
                    out_message = self.analyze_contours(contours, resized_frame)
                    if self.config['image_proc']['debug']:
                        cv2.imshow("Original", frame)
                        cv2.imshow("Cropped", resized_frame)
                        cv2.imshow("HSV", hsv_frame)
                        cv2.imshow("Mask", mask)

                    if cv2.waitKey(1000 // self.config['webcam']['fps']) & 0xFF == ord('q'):
                        break
                else:
                    out_message = {"status": "idle"}
                self.send_message(out_message)
                self.receive_message()
            self.send_message({"status": "stopped"})

        finally:
            self.cleanup()


if __name__ == "__main__":
    CONFIG_PATH = '/home/amrmgr/amr/config/opencv_config.yaml'
    TELEM_SOCKET_PATH = "ipc:///home/amrmgr/amr/tmp/vision_telem"
    CMD_SOCKET_PATH = "ipc:///home/amrmgr/amr/tmp/vision_cmd"

    vision_processor = VisionProcessor(CONFIG_PATH, TELEM_SOCKET_PATH, CMD_SOCKET_PATH)
    vision_processor.run()
