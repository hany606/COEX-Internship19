import cv2
import numpy as np
from time import time
from math import floor

cap = cv2.VideoCapture(0)

global first_time
first_time = 0

while(1):

	ret, frame = cap.read()
	cv2.imshow("Stream", frame)
	k = cv2.waitKey(1)
	if(k == ord('q')):
		first_time = time()
	print("5 - {:}".format(floor(first_time - time())))
	if(-floor(first_time - time()) == 5):
		cv2.imwrite("frame.jpg", frame)
		break
cap.release()
cv2.destroyAllWindows()
