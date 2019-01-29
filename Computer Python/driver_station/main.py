#!python2
from __future__ import print_function
import sys
import cv2
import zmq
import base64
import numpy as np

context = zmq.Context()
footage_socket = context.socket(zmq.SUB)
footage_socket.bind('tcp://*:5555')
footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))
f = open('myfile.txt', 'w')
cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)

def my_print(text):
    sys.stdout.write(str(text))
    sys.stdout.flush()

my_print("test")
while True:
    
    frame = footage_socket.recv_string()
    #my_print(frame)
    my_print("test")
    #img = base64.b64decode(frame)
    #npimg = np.fromstring(img, dtype=np.uint8)
    #source = cv2.imdecode(npimg, 1)
    #cv2.imshow("Stream", source)
    #cv2.waitKey(1)

   # except KeyboardInterrupt:
    #    cv2.destroyAllWindows()
     #   footage_socket.close()
      #  break