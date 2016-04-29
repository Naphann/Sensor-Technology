#! /opt/local/bin/python
import random

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

import cv2

PARTICLE_COUNT = 1000
# seperated distance of the references in inch
REF_DISTANCE = 24
# room width and length
BREADTH = 300
DEPTH = 300

# global
particles = np.array([], int)
wts = np.array([], int)
left_obj = [BREADTH // 2 - REF_DISTANCE // 2, DEPTH]
right_obj = [BREADTH // 2 + REF_DISTANCE // 2, DEPTH]
is_left = True
lowerb_left, lowerb_right = (0, 0, 0), (0, 0, 0)
upperb_left, upperb_right = (255, 255, 255), (255, 255, 255)


def initialize():
    """
    initialize the particle in the system
    -> list of particles
    """
    global particles
    b = np.random.randint(1, BREADTH, PARTICLE_COUNT)
    d = np.random.randint(1, DEPTH, PARTICLE_COUNT)
    particles = np.array([[b[i], d[i]] for i in range(PARTICLE_COUNT)])
    plt.clf()
    plt.figure('initialize')
    plt.clf()
    plt.plot([left_obj[0]], [left_obj[1]], 'ro')
    plt.plot([right_obj[0]], [right_obj[1]], 'ro')
    plt.plot([x[0] for x in particles], [x[1] for x in particles], 'bo')
# --


def read_sensor():
    """
    read data from img
    ->
    """
    pass
# --


def calculate_weight(mean1, sd1, mean2, sd2):
    """
    calculate weight for current particles from mean and sd
    -> change wts ...
    """
    global wts

    def cal_dist(x, obj_pos):
        temp = np.array(x) - np.array(obj_pos)
        return int(np.sqrt(np.dot(temp, temp)))

    obj1_position = np.array([left_obj[0], left_obj[1]])
    obj2_position = np.array([right_obj[0], right_obj[1]])
    distance1_list = [cal_dist(x, obj1_position) for x in particles]
    distance2_list = [cal_dist(x, obj2_position) for x in particles]
    wts = np.array([norm(mean1, sd1).pdf(distance1_list[x]) * 1000 * norm(mean2, sd2).pdf(distance2_list[x])
                    for x in range(len(distance1_list))])
    wts = wts / sum(wts)
# --


def calculate_weight_each_point(cur, normal_dist):
    """
    cur: current_point
    normal_dist: tuple (mean, sd)
    -> weight of the current point
    """
    mean, sd = normal_dist
    return norm(normal_dist).pdf(dist)
# --


def resampling():
    """
    resampling particles from weight
    -> list of possible position
    """
    global particles

    def fake_error():
        return [np.random.randint(-10, 11), np.rand / om.randint(-10, 11)]
    index = np.random.choice(PARTICLE_COUNT, PARTICLE_COUNT, p=wts)
    print(wts)
    particles = np.array([[particles[i][0] + np.random.randint(-20, 20),
                           particles[i][1] + np.random.randint(-20, 20)]
                          for i in index])
# --


def plot():
    plt.figure('after resampling')
    plt.plot([left_obj[0]], [left_obj[1]], 'ro')
    plt.plot([right_obj[0]], [right_obj[1]], 'ro')
    plt.plot([x[0] for x in particles], [x[1] for x in particles], 'bo')
    plt.axis([-10, 310, -10, 310])
    plt.draw()
    plt.show()
    print('endddd')
    return


def particle_fiter(frame):
    initialize()
    # take pictures here and calculate distance and get mean,sd
    pic_l = cv2.inRange(frame, lowerb_left, upperb_left)
    pic_r = cv2.inRange(frame, lowerb_right, upperb_right)
    pix_count_l = cv2.countNonZero(pic_l)
    pix_count_r = cv2.countNonZero(pic_r)
    print(' left count : {}, right count {}'.format(pix_count_l, pix_count_r))
    coef = np.array([1.02263387e+04,  4.26910153e+00])
    mean1 = 1 / np.sqrt(pix_count_l) * coef[0] + coef[1]
    mean2 = 1 / np.sqrt(pix_count_r) * coef[0] + coef[1]
    print(mean1, mean2)
    sd1 = 0.03 * mean1
    sd2 = 0.03 * mean2
    calculate_weight(mean1, sd1, mean2, sd2)
    resampling()
    plot()


def callback(event, x, y, flag, param):
    global img, hsv, lowerb_left, lowerb_right, upperb_left, upperb_right
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(frame, (x, y), 100, (255, 0, 0), 1)
        lowerb = hsv[y][x] - 30
        lowerb[2] -= 25
        upperb = hsv[y][x] + 30
        if upperb[0] < lowerb[0]:
            upperb[0] = 255
        if upperb[1] < lowerb[1]:
            upperb[1] = 255
        upperb[2] += 25
        if upperb[2] < lowerb[2]:
            upperb[2] = 255
        print(lowerb, upperb)
        if is_left:
            lowerb_left = lowerb
            upperb_left = upperb
        else:
            lowerb_right = lowerb
            upperb_right = upperb
        print(' hello : {} {} {} {}'.format(lowerb_left,
                                            lowerb_right, upperb_left, upperb_right))


cap = cv2.VideoCapture(0)
cv2.namedWindow('window')
cv2.setMouseCallback('window', callback)

lowerb = (0, 0, 0)
upperb = (255, 255, 255)

obj1_position = np.array([left_obj[0], left_obj[1]])
obj2_position = np.array([right_obj[0], right_obj[1]])
print(obj1_position)
print(obj2_position)
plt.ion()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if is_left:
        cv2.putText(frame, 'click to set left', (10, 500), 0, 4,
                    (255, 255, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, 'click to set right', (10, 500), 0, 4,
                    (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow('window', frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
    cmd = cv2.waitKey(20)
    # check what key the user had pressed
    if cmd == 27:  # esc
        print('program ended')
        break
    elif cmd == 32:  # spacebar
        ret, img = cap.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        particle_fiter(hsv)
    elif cmd == ord('l'):
        is_left = True
    elif cmd == ord('r'):
        is_left = False

# # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
