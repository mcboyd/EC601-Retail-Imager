import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
imgL = cv.imread('imL.png',0)
imgR = cv.imread('imR.png',0)
stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(imgL,imgR)
f= open("disparity.txt","w+")
for i in range(len(disparity)):
     f.write(str(disparity))
f.close()
plt.imshow(disparity,'gray')
plt.show()