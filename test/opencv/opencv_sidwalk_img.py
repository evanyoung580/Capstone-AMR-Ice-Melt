import cv2
import numpy as np
import yaml

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

# Convert the cropped image to HSV
hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

# BGR to HSV and max and min values
sidewalk_bgr = config['image_proc']['color_detect']['bgr_targ']
sidewalk_bgr = np.uint8([[sidewalk_bgr]])
sidewalk_hsv = cv2.cvtColor(sidewalk_bgr, cv2.COLOR_BGR2HSV)
tolerance = config['image_proc']['color_detect']['hue_tol']
hsv_min = sidewalk_hsv[0][0][0] - tolerance, \
    config['image_proc']['color_detect']['sat']['min'], config['image_proc']['color_detect']['val']['min']
hsv_max = sidewalk_hsv[0][0][0] + tolerance, \
    config['image_proc']['color_detect']['sat']['max'], config['image_proc']['color_detect']['val']['max']
hsv_min = np.array(hsv_min, dtype=np.uint8)
hsv_max = np.array(hsv_max, dtype=np.uint8)

# mask in range color values
mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if contours:
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(resized_image, (x + (w // 2), y + (h // 2)), 5, (0, 255, 0), -1)

cv2.imshow("original", image)
cv2.imshow("cropped", resized_image)
cv2.imshow("hsv", hsv_image)
cv2.imshow("mask", mask)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()