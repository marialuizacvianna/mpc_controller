import numpy as np
import cv2
from PIL import Image
import yaml
import math


def distance_squared_1d(input,output):
    #Each parabola has the form
    #input[index] + (query - index)^2

    #The grid location of the i-th parabola is parabola_idxs[i]
    parabola_idxs = np.zeros(len(input))
    parabola_idxs[0] = 0

    #The range of the i-th parabola is
    #parabola_boundaries[i] to parabola_boundaries[i + 1]
    #Initialize all of the ranges to extend to infinity
    parabola_boundaries = np.zeros(len(input) + 1)
    parabola_boundaries[0] = -float('inf')
    parabola_boundaries[1] = float('inf')

    #The number of parabolas in the lower envelope
    num_parabolas = 0

    #Compute the lower envelope over all grid cells
    #intersection_point = 0
    for idx in range(1,len(input)):
        num_parabolas = num_parabolas + 1

        while True:
            num_parabolas = num_parabolas - 1
            #The location of the rightmost parabola in the lower envelope
            parabola_idx = int(parabola_idxs[num_parabolas])
            #Compute the intersection point between the current and rightmost parabola
            #by solving for the intersection point, p:
            #
            #input[idx] + (p - idx)^2 = input[parabolaIdx] - (p - parabolaIdx)^2
            #
            # print("idx = ", idx)
            # print("parabola_idx = ", parabola_idx)
            # print("(2 * (idx - parabola_idx)) = ", (2 * (idx - parabola_idx)))
            intersection_point = ((input[idx] + idx * idx)- (input[parabola_idx] + parabola_idx * parabola_idx))/(2 * (idx - parabola_idx))
            #If the intersection point is before the boundary,
            #the rightmost parabola is not actually part of the
            #lower envelope, so decrease k and repeat.
            # print("intersection_point = ", intersection_point)
            # print("parabola_boundaries[num_parabolas] = ", parabola_boundaries[num_parabolas])
            if (intersection_point <= parabola_boundaries[num_parabolas]):
                break
        #Move to the next parabola
        num_parabolas = num_parabolas + 1

        parabola_idxs[num_parabolas] = idx
        parabola_boundaries[num_parabolas] = intersection_point
        parabola_boundaries[num_parabolas+1] = float('inf')

    parabola = 0
    for idx in range(len(input)):
        #Find the parabola corresponding to the index
        while (parabola_boundaries[parabola + 1] < idx):
            parabola = num_parabolas + 1

        #Compute the value of the parabola
        idx_dist = idx - parabola_idxs[parabola]

        output[idx] = idx_dist * idx_dist + input[int(parabola_idxs[parabola])]

    return input

map_img = np.array(Image.open('/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.png').transpose(Image.FLIP_TOP_BOTTOM))
map_img.astype(np.float64)
map_height = map_img.shape[0]
map_width = map_img.shape[1]
free_threshold = 0.6  # TODO: double check
theta = 0.0

origin_c = math.cos(theta);
origin_s = math.sin(theta);

with open('/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.yaml', 'r') as yaml_stream:
    try:
        map_metadata = yaml.safe_load(yaml_stream)
        map_resolution = map_metadata['resolution']
        origin = map_metadata['origin']
    except yaml.YAMLError as ex:
        print(ex)

map_inited = True

map = ((1. - map_img/255.).flatten().tolist())

dt = np.zeros(len(map))
print("len(map) = ", len(map))

for i in range(len(map)):
    if (0 <= map[i] and map[i] <= free_threshold):
         dt[i] = 99999;
    else:
         dt[i] = 0

#DistanceTransform::distance_2d(dt, width, height, resolution)

col_vec = np.zeros(map_height + 2)
col_dt = np.zeros(map_height + 2)

for col in range(map_width):
    col_vec[0] = free_threshold
    col_vec[map_height + 1] = free_threshold
    for row in range(map_height):
        col_vec[row + 1] = dt[row * map_width + col]

    col_vec = distance_squared_1d(col_vec, col_dt)
    for row in range(map_height):
        dt[row * map_width + col] = col_dt[row + 1]

#Transform along the rows
row_vec = np.zeros(map_width + 2)
row_dt = np.zeros(map_width + 2)

for row in range(map_height):
    row_vec[0] = free_threshold
    row_vec[map_width + 1] = free_threshold
    for col in range(map_width):
        row_vec[col + 1] = dt[row * map_width + col]

    row_vec = distance_squared_1d(row_vec, row_dt);
    for col in range(map_width):
        dt[row * map_width + col] = row_dt[col + 1]

for i in range(len(dt)):
    dt[i] = map_resolution * math.sqrt(dt[i])
# for i in range(len(dt)):
#     dt[i] = resolution * sqrt(dt[i])


# map_resolution = 0.050000
# origin = [-11.606540, -26.520793, 0.000000]
# Reading image
# font = cv2.FONT_HERSHEY_COMPLEX
# img2 = cv2.imread('/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.png', cv2.IMREAD_COLOR)

# # Reading same image in another
# # variable and converting to gray scale.
# img = cv2.imread('/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.png', cv2.IMREAD_GRAYSCALE)
#
# map_height =
#
# # Converting image to a binary image
# # ( black and white only image).
# _, threshold = cv2.threshold(img, 110, 255, cv2.THRESH_BINARY)
#
# # Detecting contours in image.
# contours, _= cv2.findContours(threshold, cv2.RETR_TREE,
#                                cv2.CHAIN_APPROX_SIMPLE)
#
# # Going through every contours found in the image.
# for cnt in contours :
#
#     approx = cv2.approxPolyDP(cnt, 0.00009 * cv2.arcLength(cnt, True), True)
#
#     # draws boundary of contours.
#     cv2.drawContours(img2, [approx], 0, (0, 0, 255), 5)
#
#     # Used to flatted the array containing
#     # the co-ordinates of the vertices.
#     n = approx.ravel()
#     i = 0
#
#     print("number points = ", n.shape)
#     # for j in n :
#     #     if(i % 2 == 0):
#     #         x = n[i]
#     #         y = n[i + 1]
#     #
#     #         # String containing the co-ordinates.
#     #         string = str(x) + " " + str(y)
#     #
#     #         if(i == 0):
#     #             # text on topmost co-ordinate.
#     #             cv2.putText(img2, "Arrow tip", (x, y),
#     #                             font, 0.5, (255, 0, 0))
#     #         else:
#     #             # text on remaining co-ordinates.
#     #             cv2.putText(img2, string, (x, y),
#     #                       font, 0.5, (0, 255, 0))
#     #     i = i + 1
#
# # Showing the final image.
# cv2.imshow('image2', img2)
#
# # Exiting the window if 'q' is pressed on the keyboard.
# if cv2.waitKey(0) & 0xFF == ord('q'):
#     cv2.destroyAllWindows()
