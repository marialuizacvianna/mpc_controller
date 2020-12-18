import numpy as np
import cv2
from PIL import Image
import yaml
import math


map_resolution = 0.050000
origin = [-11.606540, -26.520793, 0.000000]

out_contour_file = open("../out_contour_berlin.txt","w")
in_contour_file = open("../in_contour_berlin.txt","w")
# Reading image
font = cv2.FONT_HERSHEY_COMPLEX
img2 = cv2.imread('/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.png', cv2.IMREAD_COLOR)
img2 = cv2.transpose(img2);
img2 = cv2.flip(img2, 1);
# Reading same image in another
# variable and converting to gray scale.
img = cv2.imread('/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.png', cv2.IMREAD_GRAYSCALE)
img = cv2.transpose(img);
img = cv2.flip(img, 1);
# map_height =

# Converting image to a binary image
# ( black and white only image).
_, threshold = cv2.threshold(img, 0.65*255, 255, cv2.THRESH_BINARY)

# Detecting contours in image.
contours, _= cv2.findContours(threshold, cv2.RETR_TREE,
                               cv2.CHAIN_APPROX_SIMPLE)

count = 0
# Going through every contours found in the image.
for cnt in contours :
    count = count + 1
    approx = cv2.approxPolyDP(cnt, 0.000001 * cv2.arcLength(cnt, True), True)

    # draws boundary of contours.
    cv2.drawContours(img2, [approx], 0, (0, 0, 255), 5)

    # Used to flatted the array containing
    # the co-ordinates of the vertices.
    n = approx.ravel()
    i = 0

    print("number points = ", n.shape)

    for point in approx:
        x_rot = point[0][1]*map_resolution
        y_rot = point[0][0]*map_resolution

        x_trans = x_rot
        y_trans = y_rot

        x = x_trans + origin[0]
        y = y_trans + origin[1]

        if(count == 1):
            out_contour_file.writelines(str(x) + "," +  str(y) + '\n')
        else:
            in_contour_file.writelines(str(x) + "," +  str(y) + '\n')



out_contour_file.close()
in_contour_file.close()

# Showing the final image.
cv2.imshow('image2', img2)

# Exiting the window if 'q' is pressed on the keyboard.
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()
