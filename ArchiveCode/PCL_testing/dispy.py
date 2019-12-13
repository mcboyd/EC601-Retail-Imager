import numpy as np
import cv2
from matplotlib import pyplot as plt

leftImage  = cv2.imread('imL.png',0)
rightImage = cv2.imread('imR.png',0)


stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
fig = plt.figure()
disparity = stereo.compute(leftImage,rightImage)
plt.imshow(disparity,'gray')
plt.show()
plt.imsave('plot.png', disparity)