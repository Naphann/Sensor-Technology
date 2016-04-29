#! /usr/bin/python
import pathlib
import os
import cv2

preparation = False
distance = 168  
PIC_COUNT = 50

# for saving image


def save_img(num):
    global distance, preparation
    if not preparation:
        prepare_dest()
    for i in range(1, num + 1):
        ret, frame = cap.read()
        cv2.imwrite('{0}-cm-{1}.png'.format(distance, str(i).zfill(3)), frame)
    distance += 21

# preparation function


def prepare_dest():
    global preparation, distance
    # create directory for save
    col_no = 1
    cur_dir = pathlib.Path('.')
    if not (cur_dir / 'saved_img').exists():
        os.mkdir('saved_img')
    dest = cur_dir / 'saved_img'
    os.chdir('./saved_img')
    dest = pathlib.Path('.')
    for item in dest.iterdir():
        if str(item).split('-')[0] == 'collection':
            print('exists no{0}'.format(col_no))
            col_no += 1

    os.mkdir('collection-{0}'.format(col_no))
    os.chdir('collection-{0}'.format(col_no))
    preparation = True
# end of function

cap = cv2.VideoCapture(0)
print('usage: press command by the list')
print(' - press "+" to increase distance by 30cm')
print(' - press "d" to set start distance')
print(' - press spacebar to start saving picture at current distance')
print(' - press "esc" to abort the program')

# is preparation for saving destination is ready

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv2.putText(frame, 'ready {}'.format(str(distance)), (10, 500), 0, 4,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow('video', frame)
    cmd = cv2.waitKey(20)
    # check what key the user had pressed
    if cmd == 27:  # esc
        print('program ended')
        break
    elif cmd == 32:  # spacebar
        ret, frame = cap.read()
        cv2.putText(frame, 'saving ...', (10, 500),  0,
                    5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('video', frame)
        cv2.waitKey(1)
        save_img(PIC_COUNT)
    elif cmd == ord('='):
        distance += 10
        print('current distance = {0}'.format(str(distance)))
    elif cmd == ord('d'):
        distance = int(input('please input start distance'))

# # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
