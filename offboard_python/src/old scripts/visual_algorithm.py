#!/usr/bin/env python

import cv2
from ProcessFrame import ProcessFrame

cap = cv2.VideoCapture('video_puta.mp4')
frame_processing = ProcessFrame()

while True:
    _, frame = cap.read()
    print frame_processing.run(frame, show=False)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
