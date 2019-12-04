import numpy as np
import cv2 as cv
import pickle
import sys
import requests

if len(sys.argv)<=1:
	sys.exit("Requires 2 parameters 1: index file name, 2: image file name to describe and match")

indexFile = sys.argv[1]  # test with 'large'
inputImg = sys.argv[2]  # test with 'surface7.jpg'

# Load necessary pickle files
file_des = open('DB/p128_des_'+indexFile+'.obj', 'rb')  # SIFT obj of features
des_t = pickle.load(file_des)

file_imgName = open('DB/p128_imgName_'+indexFile+'.obj', 'rb')  # Index description of images in des
imgName = pickle.load(file_imgName)

# Load saved index for search; all these steps are required to make it the right type...
o = cv.flann_Index()
mytype = type(o)
flann = mytype()
flann.load(des_t,'DB/p128_des_'+indexFile+'_idx.obj')  # Insert index object file name here


# Setup SIFT feature extraction process
sift = cv.xfeatures2d.SIFT_create()

oriimg = cv.imread(inputImg)  # Populate with image to search for
#img = cv.resize(oriimg,None,fx=0.2,fy=0.2)
img = oriimg
imgrgb= cv.cvtColor(img,cv.COLOR_BGR2RGB)
kp = sift.detect(imgrgb,None)
kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value(bigger is better)
kps,des2 = sift.compute(imgrgb,kps)


# Search for image in FLANN index loaded above
idx, dist = flann.knnSearch(des2, 1, params={})

# "idx" contains the 128 indexes of the best matches from the input image
# Divide by 128 to know which image corresponds to each match (since each image in index has 128 feature points)
l = idx//128
counts = np.bincount(l.flatten())  # Flatten the (128,1) idx and count the image indexes returned;
match = imgName[np.argmax(counts)]  # Match is the image in the index (based on which image is matched in the 128 indexes the most)
# In match: (img file name, start index, end index, product id)
print(match[3])  # Prints product ID
print(match[0])  # TESTING - REMOVE FOR PRODUCTION # and matching file name

# Send GET request with product id to GUI
res = requests.get('http://127.0.0.1:8081/process_get?prodid=' + match[3])
print(res)  # Should be 200 OK