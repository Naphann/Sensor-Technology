#! /opt/local/bin/python
import os
import pathlib
import sys

import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import numpy as np

import cv2
#--function--#
def create_sensor_model():
    #create output file
    data = {} 
    output = open(output_filename + '.txt', 'w')
    output.write('range of detected hsv-color {0} - {1}\n'.format(lowerb, upperb))
    #Count pixel from image list and store in data as dict
    for img in img_list:
        if not img.split('.')[1] == 'png':
            continue
        hsv = cv2.imread(img)
        hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV_FULL)
        key = img.split('-')[0]
        if not key in data:
            data[key] = []
        detected = cv2.inRange(hsv, lowerb, upperb)
        data[key].append(cv2.countNonZero(detected))
    #write data to file 
    keys = sorted(list(data.keys()))
    for i in keys:
        output.write('{0}cm with mean={1}, sd={2} - {1}'.format(i, np.mean(data[i]), np.std(data[i],ddof=1), data[i][0]))
        for j in data[i][1:]:
            output.write(', {0}'.format(j))
        output.write('\n')    
    output.close()
    print('finish')
    cv2.destroyAllWindows()
    exit()
        
def create_sensor_model_deprecated():
    global img, hsv, upperb, lowerb
    data = np.array([])
    file_list = []
    cur_dir = pathlib.Path('.')
    for item in cur_dir.iterdir(): 
        file_list.append(str(item))
    for file in file_list:
        if file.split('.')[1] == 'png' and file.split('-')[0] == '60':
            pic = cv2.imread(file)
            pic = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV_FULL)
            pic = cv2.inRange(pic, lowerb, upperb)
            data = np.append(data, cv2.countNonZero(pic))
    print(data)
    plt.plot(data, [1] * len(data), 'ro')
    plt.show()
 
def callback(event, x, y, flag, param):
    global img, hsv, upperb, lowerb
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(img, (x, y), 100, (255, 0, 0), 1)
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
        create_sensor_model()
#--function--#

#Get command line arguments
if len(sys.argv) != 3:
    print('usage: python $create_sensor_model$ <collection-number> <color-name-for-output-file>')
    exit()
collection_number = sys.argv[1]
output_filename   = sys.argv[2]

#opening directory
try:
    os.chdir('saved_img/collection-{0}'.format(collection_number))
except exception:
    print('directory not exist')
    exit()
img_list = os.listdir()
img_list = img_list[1:]
# show first picture to get color range
img = cv2.imread(img_list[0])
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
cv2.namedWindow('window')
cv2.setMouseCallback('window', callback)

lowerb = (0, 0, 0)
upperb = (255, 255, 255)
while True:
    output = cv2.inRange(hsv, lowerb, upperb)
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, (10, 10))
    cv2.imshow('window', img)
    cmd = cv2.waitKey(100)
    if cmd == 27:
        break

cv2.destroyAllWindows()
