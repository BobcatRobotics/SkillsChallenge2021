from cscore import CameraServer
import cv2
import numpy as np

# Color filtering params
hueLower = 16
hueUpper = 43
satLower = 49
satUpper = 255
valLower = 82
valUpper = 255

lowerBound = np.array([hueLower, satLower, valLower])
upperBound = np.array([hueUpper, satUpper, valUpper])

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
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_frame = cv2.inRange(hsv, lowerBound, upperBound)

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

def writeDirection(direction: int, frame):
    position = (50, 0)
    if direction == RIGHT:
        frameWithText = cv2.putText(
            frame, 
            "RIGHT", 
            position, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (255, 255, 255, 255), 
            1
        )
        return frameWithText
    elif direction == LEFT:
        frameWithText = cv2.putText(
            frame, 
            "LEFT", 
            position, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (255, 255, 255, 255), 
            1
        )
        return frameWithText
    else:
        print("Invalid Direction...")
        return frame



if __name__ == '__main__':

    cs = CameraServer.getInstance()
    cs.enableLogging()

    mainCamera = cs.startAutomaticCapture()
    mainCamera.setResolution(1920, 1080)

    outputSink = cs.putVideo("OuputStream", 1920, 1080)

    frame = None

    while True:
        time, frame = outputSink.grabFrame(frame)

        if time == 0:
            outputSink.notifyError(outputSink.getError())
            continue

        coords = detectBalls(frame)

        direction = chooseDirection(coords, frame)

        frameWithText = writeDirection(direction, frame)

        outputSink.putFrame(frameWithText)

        print(direction)