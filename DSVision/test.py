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

# Directions
LEFT = 0
RIGHT = 1

def nothing(x):
    print(f"trackbar value : {x}")

def detectBalls(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_frame = cv2.inRange(hsv, lowerBound, upperBound)

    # yellow_frame = cv2.medianBlur(yellow_frame, 5)

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
            (255, 0, 0), 
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
            (255, 0, 0), 
            1
        )
        return frameWithText
    else:
        print("Invalid Direction...")
        return frame


if __name__ == '__main__':
    
    capIndexList = []

    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            capIndexList.append(i)
            cap.release()
        else:
            break

    for capIndex in capIndexList:
        print(capIndex)
    print(f"Number of cameras: {len(capIndexList)}")

    capIndexChosen = int(input("Which camera? : "))
    
    '''
    cv2.namedWindow('thresh')
    cv2.createTrackbar('hueUpper', 'thresh', 255, 255, nothing)
    cv2.createTrackbar('satUpper', 'thresh', 255, 255, nothing)
    cv2.createTrackbar('valUpper', 'thresh', 255, 255, nothing)
    cv2.createTrackbar('hueLower', 'thresh', 0, 255, nothing)
    cv2.createTrackbar('satLower', 'thresh', 0, 255, nothing)
    cv2.createTrackbar('valLower', 'thresh', 0, 255, nothing)
    '''
    cap = cv2.VideoCapture(capIndexChosen)

    while True:
        ret, frame = cap.read()
        '''
        hueUpper = cv2.getTrackbarPos('hueUpper', 'thresh')
        satUpper = cv2.getTrackbarPos('satUpper', 'thresh')
        valUpper = cv2.getTrackbarPos('valUpper', 'thresh')
        hueLower = cv2.getTrackbarPos('hueLower', 'thresh')
        satLower = cv2.getTrackbarPos('satLower', 'thresh')
        valLower = cv2.getTrackbarPos('valLower', 'thresh')
        '''
        if ret:
            #cv2.imshow('test', frame)
            coords = detectBalls(frame)
            direction = chooseDirection(coords, frame)
            frameWithText = writeDirection(direction, frame)
            cv2.imshow('ouput', frameWithText)
            print(direction)

            '''
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            '''
            '''yellow_frame = frame[
                hueLower:hueUpper, 
                satLower:satUpper, 
                valLower:valUpper
            ]'''
            '''
            lowerBound = np.array([hueLower, satLower, valLower])
            upperBound = np.array([hueUpper, satUpper, valUpper])

            yellow_frame = cv2.inRange(hsv, lowerBound, upperBound)

            yellow_frame = cv2.medianBlur(yellow_frame, 5)
            cv2.imshow('thresh', yellow_frame)
            cv2.imshow('withColor', cv2.bitwise_and(frame, frame, mask=yellow_frame))
            print(f"lower : {lowerBound}")
            print(f"upper : {upperBound}")
            '''
        if cv2.waitKey(1)&0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            break