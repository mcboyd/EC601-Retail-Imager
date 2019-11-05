import numpy as np
import cv2 as cv
import pickle
import os

dirName = '../VDMSDataset/images/'  # Sample images added to make indexes large

# Get the list of all files in directory tree at given path above
listOfFiles = list()
for (dirpath, dirnames, filenames) in os.walk(dirName):
    listOfFiles += [os.path.join(dirpath, file) for file in filenames]

# Setup Sift
sift = cv.xfeatures2d.SIFT_create()

imgName=[]  # Image info index; populated as images are described and added to description array
des_t = None  # Image description array; each image added should be 128x128 array of floats
des_i = -1  # Used to track index values for each image's beginning and ending index number in des_t
zerow = np.zeros((1, 128),dtype="float32")  # Row of zeros just in case sample image doesn't include 128 features

for x in listOfFiles:
	oriimg = cv.imread(x)  # Sample images are all small, so no resize necessary
	imgrgb= cv.cvtColor(oriimg,cv.COLOR_BGR2RGB)
	kp = sift.detect(imgrgb,None)
	kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value (bigger is better)
	kps,des = sift.compute(imgrgb,kps)
	
	while len(des)<128:  # If fewer than 128 features in the current image, pad des with rows of zeros upto 128 rows
		des = np.concatenate([des, zerow])

	imgName.append([x,des_i+1,des_i+len(des),0])  # Adds image info, indexes, and product id ("0" for sample images) to image detail matrix
	des_i+=len(des)
	
	if des_t is None:
		des_t = des  # If this is the first image processed, set des_t to equal it
	else:
		des_t = np.concatenate((des_t,des))  # Else concatenate to existing des_t

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# PUT REAL IMAGES TO ADD TO THIS INDEX IN THE "files" VARIABLE BELOW
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
files = ["surface1.jpg","surface2.jpg","surface3.jpg","surface4.jpg","surface5.jpg","surface6.jpg","surface7.jpg","surface8.jpg","surface9.jpg",]

for x in files:
	oriimg = cv.imread('../VDMSDataset/dataset/img/'+x)
	img = cv.resize(oriimg,None,fx=0.2,fy=0.2)  # Resize the images to a more reasonable size (default size is 1920x1080, which is large)
	imgrgb= cv.cvtColor(img,cv.COLOR_BGR2RGB)
	kp = sift.detect(imgrgb,None)
	kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value(bigger is better)
	kps,des = sift.compute(imgrgb,kps)
	
	imgName.append([x,des_i+1,des_i+len(des),0])
	des_i+=len(des)
	if des_t is None:
		des_t = des
	else:
		des_t = np.concatenate((des_t,des))

# Save des_t and imgName to object files on disk for later use
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# UPDATE FILE NAMES BELOW TO MATCH NAMING SCHEME AND CONTENTS
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
object_des = des_t
file_des = open('p128_des_large.obj', 'wb')
pickle.dump(object_des, file_des)
object_imgName = imgName
file_imgName = open('p128_imgName_large.obj', 'wb')
pickle.dump(object_imgName, file_imgName)