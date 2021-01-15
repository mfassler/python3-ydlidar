
import numpy as np
import cv2


COLOR_YELLOW = np.array([153,255,255])
COLOR_BLACK = np.array([50,50,50])


def make_map(width, height, grid_spacing):
    '''
    width : int
        pixel width of map
    height : int
        pixel height of map
    grid_spacing : int
        pixels per meters
    '''
    _map = np.ones((height, width, 3), np.uint8) * 255

    # Draw the grid:
    for x in np.arange(grid_spacing, width, grid_spacing):
        cv2.line(_map, (x, 0), (x, height), (255,200,150), 2)
    for y in np.arange(grid_spacing, height, grid_spacing):
        cv2.line(_map, (0, y), (width, y), (255,200,150), 2)

    # Radial gridlines:
    max_radius = np.sqrt(height**2 + (width/2)**2)
    x_center = int(round(width/2))
    y_center = int(round(height/2))
    for r in np.arange(grid_spacing, max_radius, grid_spacing):
        cv2.circle(_map, (x_center, y_center), int(r), (255,200,150), 2)

    # The position of the rover:
    #cv2.rectangle(_map, (x_center-25, height-10), (x_center+25, height), (0,0,0), -1)
    return _map





def make_yellow_avoidance_area(img, PIXELS_PER_METER):
    vehicle_width = 0.48  # meters, float
    max_width = 0.75  # meters, float
    max_distance = 1.5  # meters, float

    # Convert these real, float values into pixels on the radar screen:
    h_center = 400  # pixel center of img
    v_center = 400  # pixel center of img

    bottom_left = h_center - int(round(vehicle_width * PIXELS_PER_METER / 2))
    bottom_right = h_center + int(round(vehicle_width * PIXELS_PER_METER / 2))

    max_left = h_center - int(round(max_width * PIXELS_PER_METER / 2))
    max_right = h_center + int(round(max_width * PIXELS_PER_METER / 2))

    v_top = v_center - int(round(max_distance * PIXELS_PER_METER))

    pts = np.array([
        [bottom_left, v_center],
        [max_left, 300],
        [max_left, v_top],
        [max_right, v_top],
        [max_right, 300],
        [bottom_right, v_center],
    ], np.int32)
    pts.reshape((-1, 1, 2))
    cv2.fillPoly(img, [pts], (153, 255,255))  # TODO:  why can't I put "COLOR_YELLOW" here?


def make_black_avoidance_area(img, PIXELS_PER_METER):
    vehicle_width = 0.48  # meters, float
    max_distance = 0.75 # meters, float
    k_w = int(round(vehicle_width * PIXELS_PER_METER / 2))
    k_h = int(round(max_distance * PIXELS_PER_METER))

    cv2.ellipse(img, (400, 400), (k_w, k_h), 0, 0, -180, (50, 50, 50), -1)



