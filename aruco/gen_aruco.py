import cv2
import random
# Select the predefined dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

#Loop generate markers from randomly 5 numbers from 23 to 49 with a size of 300x300 pixels and a border of 1 bit
for i in range(5):
    marker_id = random.randint(23, 49)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, id=marker_id, sidePixels=300, borderBits=1)
    cv2.imwrite(f"marker_{marker_id}.png", marker_img)