import cv2
import numpy as np
import yaml
def make_line_length(longest):
    def line_length(line):
        x1, y1, x2, y2 = line[0] 
        angle = np.arctan2(y2 - y1, x2 - x1) * (180 / np.pi)
        ang_excl = config['image_proc']['line_detect']['line']['ang_excl']
        if ((angle > ang_excl) and (angle < 180 - ang_excl)) or \
            ((angle < -ang_excl) and (angle > -(180 - ang_excl))):
            if longest is not None:
                x1ll, _, x2ll, _ = longest[0]
                x_gap = config['image_proc']['line_detect']['line']['x_gap']
                x2d = abs(x2-x2ll) > x_gap
                x1d = abs(x1-x1ll) > x_gap
                if (x2d and x1d):
                    length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                else:
                    length = 0
            else:
                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        else: 
            length = 0
        return length
        
    return line_length
        

with open('/home/amrmgr/amr/config/opencv_config.yaml', 'r') as config_file:
    config = yaml.safe_load(config_file)

directory_path = "/home/amrmgr/amr/test/opencv/opencv_test_media/"
image_filename = "Lawn-Edging.jpg"

image_path = directory_path + image_filename

image = cv2.imread(image_path)
if image is None:
    print("Error: Unable to load the image.")

# Crop image
height, width, _ = image.shape
top_crop = int(height * config['image_proc']['crop']['top'])
bottom_crop = int(height * config['image_proc']['crop']['bottom'])
cropped_image = image[top_crop:bottom_crop, :]

# resize image
k_rescale = config['image_proc']['rescale']
height, width, _ = cropped_image.shape
resized_image = cv2.resize(cropped_image, (width * k_rescale, height * k_rescale))

gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
blur_image = cv2.blur(gray_image, (5, 5))
edge_image = cv2.Canny(blur_image, config['image_proc']['line_detect']['canny']['low'], 
                       config['image_proc']['line_detect']['canny']['low'])
lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180,
                        config['image_proc']['line_detect']['hough']['thresh'], 
                        minLineLength=config['image_proc']['line_detect']['hough']['min_len'], 
                        maxLineGap=config['image_proc']['line_detect']['hough']['max_gap'])
if lines is not None:
    longest = make_line_length(None)
    long_line = max(lines, key=longest)
    sec_longest = make_line_length(long_line)
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
        left_edge = ((np.average([x1ll, x2ll]) / width)) - 1
        right_edge = ((np.average([x1rl, x2rl]) / width)) - 1
        center = np.average([left_edge, right_edge])

        print(f"Left edge: {left_edge}, Right edge: {right_edge}, Center: {center}")
            
    if sec_long_line_len > 0:
        x1sl, y1sl, x2sl, y2sl = sec_long_line[0]
    
    cv2.line(resized_image, (x1ll, y1ll), (x2ll, y2ll), (0, 0, 255), 4)
    cv2.line(resized_image, (x1rl, y1rl), (x2rl, y2rl), (255, 0, 0), 4)

cv2.imshow("original", image)
cv2.imshow("cropped", resized_image)
cv2.imshow("hsv", gray_image)
cv2.imshow("mask", edge_image)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()