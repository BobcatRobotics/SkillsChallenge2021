import cv2
import numpy as np

# Color filtering params
hueLower = 52
hueUpper = 73
satLower = 62
satUpper = 100
valLower = 38
valUpper = 73

# Hough Circles Parameters
minDist = 1000
pr1 = 255
pr2 = 10
minR = 0
maxR = 160

port = ''

# Directions
LEFT = 0
RIGHT = 1

def detectBalls(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_frame = frame[
        hueLower:hueUpper, 
        satLower:satUpper, 
        valLower:valUpper
    ]

    yellow_frame = cv2.medianBlur(yellow_frame, 5)

    circles_temp = cv2.HoughCircles(yellow_frame, cv2.HOUGH_GRADIENT, 1, minDist, param1=pr1, param2=pr2, minRadius=minR, maxRadius=maxR)

    if circles_temp is None:
        return [-1, -1, -1]
    
    circles = np.uint16(np.around(circles_temp))

    maxRadiusFound = -1
    circleCoords = []
    for (x, y, r) in circles[0]:
        if r > maxRadiusFound:
            maxRadiusFound = r
            circleCoords = [x, y]
    
    return circleCoords

def chooseDirection(coords, frame):
    height, width = frame.shape[:2]
    if coords[0] < width/2:
        return LEFT
    else:
        return RIGHT

if __name__ == '__main__':

    cap = cv2.VideoCapture(port)

    while cap.isOpened():
        ret, frame = cap.read()

        coords = detectBalls(frame)

        direction = chooseDirection(coords, frame)