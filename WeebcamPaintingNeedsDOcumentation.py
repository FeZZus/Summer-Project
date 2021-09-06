import numpy as np
import cv2
from collections import deque
from PIL import Image
import PIL


rainbow = False
blue = True
red = False
green = False
yellow = False

count = 0
rancount = 4

# this helps to smooth the colours by creating a grid which will later be used to smooth out the image of the blue circle
kernel = np.ones((5, 5), np.uint8)

# deques are used here to act as an array which can be easily added to from both ends later on. They are used to store the points being drawn on the screen of each colour
bluepoints = [deque(maxlen=512)]
greenpoints = [deque(maxlen=512)]
redpoints = [deque(maxlen=512)]
yellowpoints = [deque(maxlen=512)]

# we also need to add an index for these which will be independent and will be used later to store the values of points
bindex = 0
gindex = 0
rindex = 0
yindex = 0

colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)]
cindex = 0

raingrad = [(229, 0, 51), (229, 0, 51), (197, 0, 72), (171, 0, 90), (144, 0, 107), (112, 0, 128), (85, 0, 145),
            (59, 0, 162), (38, 6, 176), (21, 10, 188), (14, 23, 195), (11, 40, 199), (8, 61, 205), (5, 78, 209),
            (2, 100, 215), (1, 119, 219), (0, 142, 223), (0, 162, 226), (0, 186, 230), (0, 206, 233), (0, 236, 238),
            (0, 241, 255), (212, 216, 35), (77, 170, 177), (118, 124, 143), (160, 78, 109), (195, 39, 80)]
bgrad = [(255, 108, 0), (248, 85, 11), (240, 60, 22), (233, 40, 32), (227, 22, 41), (221, 1, 50), (200, 1, 45),
         (174, 1, 29), (153, 1, 34), (133, 1, 29), (115, 1, 25), (132, 24, 15), (163, 45, 11), (192, 65, 8),
         (224, 87, 4), ]
ggrad = [(0, 216, 0), (2, 207, 0), (4, 198, 0), (6, 187, 0), (8, 179, 0), (10, 170, 0), (8, 157, 3), (6, 140, 6),
         (4, 125, 9), (2, 112, 12), (0, 99, 15), (0, 122, 12), (0, 143, 9), (0, 169, 6), (0, 193, 3)]
rgrad = [(34, 34, 255), (27, 27, 255), (20, 20, 255), (14, 14, 255), (7, 7, 255), (0, 0, 255), (0, 0, 241), (0, 0, 266),
         (0, 0, 209), (0, 0, 194), (0, 0, 179), (7, 7, 196), (13, 13, 210), (20, 20, 225), (27, 27, 240)]
ygrad = [(0, 230, 255), (0, 238, 255), (0, 247, 255), (0, 255, 255), (0, 249, 249), (0, 243, 243), (0, 237, 237),
         (0, 233, 247)]
vcol = bgrad[rancount]

# first we need to create a blank white image np.zeroes((height, width, 3))
painting = np.zeros((636, 636, 3), dtype="uint8")
# then we need to create the colour boxes to change the colours when you are drawing
# this creates a black outline
painting[0:, :, :] = 0
painting = cv2.rectangle(painting, (40, 0), (140, 65), (5, 5, 5), -1)
painting = cv2.rectangle(painting, (160, 0), (255, 65), (5, 5, 5), -1)
painting = cv2.rectangle(painting, (275, 0), (370, 65), (5, 5, 5), -1)
painting = cv2.rectangle(painting, (390, 0), (485, 65), (5, 5, 5), -1)
painting = cv2.rectangle(painting, (505, 0), (600, 65), (5, 5, 5), -1)

# 0 is used in the video capture to tell it to read from the webcam
camera = cv2.VideoCapture(0)

# Keep looping
while True:
    # Grab the current painting
    (grabbed, frame) = camera.read()
    # this allows the image to be flipped so that the resulting image from the window will not be mirrored
    frame = cv2.flip(frame, 1)
    # this sets the colour of the video feed to be correct
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # this will add the paint interface we created before over the top of each frame
    frame = cv2.rectangle(frame, (40, 1), (140, 65), (122, 122, 122), -1)
    frame = cv2.rectangle(frame, (160, 1), (255, 65), colours[0], -1)
    frame = cv2.rectangle(frame, (275, 1), (370, 65), colours[1], -1)
    frame = cv2.rectangle(frame, (390, 1), (485, 65), colours[2], -1)
    frame = cv2.rectangle(frame, (505, 1), (600, 65), colours[3], -1)
    cv2.putText(frame, "ERASE", (53, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
    cv2.putText(frame, "BLUE", (178, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
    cv2.putText(frame, "GREEN", (285, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
    cv2.putText(frame, "RED", (414, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
    cv2.putText(frame, "YELLOW", (510, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (150, 150, 150), 2)

    # this will end the loop when the video has ended and where no more frames are grabbed
    if not grabbed:
        break

    # now the webcam is up and running, we must look for a blue colour object in each frame which uses inrange() method to find the image
    # this will be done using the rfinder variable

    #                   input
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    red_finder = mask1 + mask2

    # now we have found blue on the webcam feed, we need to refine it using eroding, dilation and morphology gradietns. i have iterated it twice to allows for a smoother mask
    red_finder = cv2.erode(red_finder, kernel, iterations=2)

    # this will attempt to remove background noise from the image
    red_finder = cv2.morphologyEx(red_finder, cv2.MORPH_OPEN, kernel)

    # finally expand the areas of blue in the hopefully now noisless image
    red_finder = cv2.dilate(red_finder, kernel, iterations=1)

    # blueLower = np.array([100, 100, 100])
    # blueUpper = np.array([140,255,255])
    # blueMask = cv2.inRange(hsv, blueLower, blueUpper)
    # blueMask = cv2.erode(blueMask, kernel, iterations=2)
    # blueMask = cv2.morphologyEx(blueMask, cv2.MORPH_OPEN, kernel)
    # blueMask = cv2.dilate(blueMask, kernel, iterations=1)

    # lastly we try to find contours of the image to  recognise the shape and finally apply the mask
    # source image      #retrieval mode  #contour approximation method - this findsonly the simplest corners of the image found to make it run faster
    (contours, _) = cv2.findContours(red_finder.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    center = None

    # we then need to see if any contours were found so:
    if len(contours) > 0:
        # we then try to find the largest contour which will be the circles ones, Reverse is set to true to make it a descending order so the highes tarea will be at the start
        single_contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
        # CHECK IF CAN REMOVE BRACKETS, ALSO CHANGE single_contour AND contours
        # from here, we need to find the minimum radius of a circle which can fit round the blue shape and its co-ords
        ((x, y), radius) = cv2.minEnclosingCircle(single_contour)
        # then we can finally draw the line around the frame
        #           image, center,          radius         colour   thickness
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

        # from here we need to find the center of the circle to draw form this point. this was quite challenging so i had to do a lot of research into finding the center of a circle using opencv
        M = cv2.moments(single_contour)
        center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

        # this allows you to choose your colour when you move the blue circle upwards
        if center[1] <= 65:
            if 40 <= center[0] <= 140:  # Clear All - this resets all the values
                bluepoints = [deque(maxlen=512)]
                greenpoints = [deque(maxlen=512)]
                redpoints = [deque(maxlen=512)]
                yellowpoints = [deque(maxlen=512)]

                bindex = 0
                gindex = 0
                rindex = 0
                yindex = 0

                painting[0:, :, :] = 0
                painting = cv2.rectangle(painting, (40, 0), (140, 65), (5, 5, 5), -1)
                painting = cv2.rectangle(painting, (160, 0), (255, 65), (5, 5, 5), -1)
                painting = cv2.rectangle(painting, (275, 0), (370, 65), (5, 5, 5), -1)
                painting = cv2.rectangle(painting, (390, 0), (485, 65), (5, 5, 5), -1)
                painting = cv2.rectangle(painting, (505, 0), (600, 65), (5, 5, 5), -1)

            # when this will creat regions for each colour which will be decided when moved upon
            elif 160 <= center[0] <= 255:
                cindex = 0  # this is for the blue box
                blue = True
                green = False
                red = False
                yellow = False
            elif 275 <= center[0] <= 370:
                cindex = 1  # this is for the green
                green = True
                blue = False
                yellow = False
                red = False
            elif 390 <= center[0] <= 485:
                cindex = 2  # this is for the red box
                red = True
                blue = False
                yellow = False
                green = False
            elif 505 <= center[0] <= 600:
                cindex = 3  # this is for the rellow box
                yellow = True
                green = False
                blue = False
                red = False
            elif 620 <= center[0] <= 636:
                rainbow = True
                green = False
                blue = False
                red = False
                yellow = False
        else:  # if it is not in the region of colour changing area, it will continue to draw points
            if cindex == 0:
                bluepoints[bindex].appendleft(center)
            elif cindex == 1:
                greenpoints[gindex].appendleft(center)
            elif cindex == 2:
                redpoints[rindex].appendleft(center)
            elif cindex == 3:
                yellowpoints[yindex].appendleft(center)
    # this will append the next deque when the circle is hidden to get ready to draw the next instance as the index increase by 1
    # dequeues allow for both co-ordinates to be stored per value
    else:
        bluepoints.append(deque(maxlen=512))
        bindex += 1
        greenpoints.append(deque(maxlen=512))
        gindex += 1
        redpoints.append(deque(maxlen=512))
        rindex += 1
        yellowpoints.append(deque(maxlen=512))
        yindex += 1
    # this will finally draw between the points found above
    points = [bluepoints, greenpoints, redpoints, yellowpoints]

    # Experiment with removing the loops
    # this code will draw a line between each points listed in thte points list by going through them one by one and drawing the line on top of each frame. it will go through each deque for each colour to find the points that have been drawn with them
    for i in range(len(points)):
        # this will repeat 4 times for each colour
        for j in range(len(points[i])):
            # this will repeat for the amount of points in each colour
            for k in range(1, len(points[i][j])):
                # this will repeat for the amount of co-ords in the deque

                l = 0
                m = 1
                n = 2
                o = 3
                thickness = 2
                endx = points[i][j][k][l]
                endy = points[i][j][k][m]
                startx = points[i][j][k - 1][l]
                starty = points[i][j][k - 1][m]

                bx = 318 - (endx - 318)
                by = 236 - (endy - 236)

                fx = 318 - (startx - 318)
                fy = 236 - (starty - 236)

                xthic = 83
                ythic = -83
                ma = starty + xthic
                mb = startx + ythic
                mc = endy + xthic
                md = endx + ythic

                me = points[i][j][k - 1][m] + xthic
                mf = fx + ythic
                mg = endy + xthic
                mh = bx + ythic

                mi = fy + xthic
                mj = points[i][j][k - 1][l] + ythic
                mk = by + xthic
                ml = endx + ythic

                mm = fy + xthic
                mn = fx + ythic
                mo = by + xthic
                mp = bx + ythic
                del points[i][j][k]

                ting = rancount
                print(blue, green, red, yellow)
                if rainbow:
                    if rancount >= 26:
                        rancount = 0
                    vcol = raingrad[rancount]
                if blue:
                    if rancount >= 14:
                        rancount = 0
                    vcol = bgrad[rancount]
                if green:
                    if rancount >= 13:
                        rancount = 0
                    vcol = ggrad[rancount]
                if red:
                    if rancount >= 14:
                        rancount = 0
                    vcol = rgrad[rancount]
                if yellow:
                    if rancount >= 7:
                        rancount = 0
                    vcol = ygrad[rancount]

                fmn = mn + 83
                fmp = mp + 83
                fmb = mb + 83
                fmd = md + 83
                fmf = mf + 83
                fmh = mh + 83
                fmj = mj + 83
                fml = ml + 83
                fstarty = starty + 83
                fendy = endy + 83
                ffy = fy + 83
                fby = by + 83

                painting = cv2.line(painting, (mm, fmn), (mo, fmp), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (ma, fmb), (mc, fmd), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (me, fmf), (mg, fmh), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (mi, fmj), (mk, fml), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (startx, fstarty), (endx, fendy), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (fx, fstarty), (bx, fendy), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (fx, ffy), (bx, fby), vcol, thickness, cv2.LINE_AA)
                painting = cv2.line(painting, (startx, ffy), (endx, fby), vcol, thickness, cv2.LINE_AA)

                frame = cv2.line(frame, (mm, mn), (mo, mp), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (ma, mb), (mc, md), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (me, mf), (mg, mh), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (mi, mj), (mk, ml), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (startx, starty), (endx, endy), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (fx, starty), (bx, endy), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (fx, fy), (bx, by), vcol, thickness, cv2.LINE_AA)
                frame = cv2.line(frame, (startx, fy), (endx, by), vcol, thickness, cv2.LINE_AA)

                count += 1
                if count == 3:
                    rancount += 1
                    count = 0
                # cv2.putText(painting, "ERASE", (53, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2, cv2.LINE_4)
                # cv2.putText(painting, "BLUE", (178, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
                # cv2.putText(painting, "GREEN", (285, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
                # cv2.putText(painting, "RED", (414, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (255, 255, 255), 2)
                # cv2.putText(painting, "YELLOW", (510, 38), cv2.FONT_HERSHEY_PLAIN, 1.4, (150, 150, 150), 2)

                cv2.drawMarker(frame, (center[0], center[1]), (255, 255, 255), thickness=2)
                save = painting.copy()
    cv2.imshow("live", frame)
    cv2.imshow("painting", painting)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.imwrite("C:/Users/ammar/Downloads/test.jpg", save)
camera.release()
cv2.destroyAllWindows()
