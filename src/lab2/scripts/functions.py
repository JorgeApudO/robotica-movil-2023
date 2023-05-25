import numpy as np
import cv2 as cv

def sawtooth(rad):
    return (rad - np.pi) % (2*np.pi) - np.pi


def min_rotation_diff(goal, actual):
    if abs(goal - actual) > np.pi:
        if actual < 0:
            return 2*np.pi + actual - goal
        else:
            return actual - goal - 2*np.pi
    else:
        return actual - goal


def pos_y_pendiente(pos):
    x1, y1, x2, y2 = pos
    if x2-x1 == 0 or y2-y1 == 0:
        return np.zeros(2)
    return np.array(x1, x2)


def filter_color(rgb_img, filter_hue):
    hsv = cv.cvtColor(rgb_img, cv.COLOR_BGR2HSV)

    if filter_hue - 10 < 0:
        lower = 180 + filter_hue - 10
    else:
        lower = filter_hue - 10

    if filter_hue + 10 > 180:
        upper = filter_hue + 10 - 180
    else:
        upper = filter_hue + 10

    lower_limit = np.array([lower, 100, 50])
    upper_limit = np.array([upper, 255, 255])

    mask = cv.inRange(hsv, lower_limit, upper_limit)

    return mask


def get_image_means(depth_image):
    # Left depth from image
    depth_left = depth_image[100:300, :30]
    depth_left = depth_left[depth_left <= 1000]
    # Right depth from image
    depth_right = depth_image[100:300, -30:]
    depth_right = depth_right[depth_right <= 1000]
    # Center depth from image
    depth_center = depth_image[100:300, 150:-150]
    depth_center = depth_center[depth_center <= 1000]

    prom_center = float(np.nanmean(depth_center))
    prom_left = float(np.nanmean(depth_left))
    prom_right = float(np.nanmean(depth_right))

    return prom_left, prom_right, prom_center


def get_centers(mask):
    bilateral = cv.bilateralFilter(mask, 9, 75, 75)
    median = cv.medianBlur(bilateral, 7)
    M = cv.moments(median)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    else:
        return (-1, -1)


def get_mask(img, hue):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_limit = np.array([hue - 10, 100, 50])
    upper_limit = np.array([hue + 10, 255, 255])

    mask = cv.inRange(hsv, lower_limit, upper_limit)

    return mask