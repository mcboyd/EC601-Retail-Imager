import numpy as np
import cv2 as cv
import pickle

file_des = open('p128_des.obj', 'rb')
des_t = pickle.load(file_des)

file_imgName = open('p128_imgName.obj', 'rb')
imgName = pickle.load(file_imgName)

sift = cv.xfeatures2d.SIFT_create()

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

flann_index = cv.flann_Index

oriimg = cv.imread('../VDMSDataset/dataset/img/surface6.jpg')
#print('../VDMSDataset/dataset/img/'+x)
img = cv.resize(oriimg,None,fx=0.2,fy=0.2)
#print(img.shape)
imgrgb= cv.cvtColor(img,cv.COLOR_BGR2RGB)
print("keypoints starting for 6")
kp = sift.detect(imgrgb,None)
kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value(bigger is better)
print("keypoints done for 6")
kps,des2 = sift.compute(imgrgb,kps)
print(des2.shape)

print("Building index")
flann = flann_index(des_t, index_params)
print("Searching...")
idx, dist = flann.knnSearch(des2, 1, params={})

print(len(idx))
print(len(dist))

for i in range(len(idx)):
	#print(idx[0][0])
	h = idx[i,0]
	#print(h)
	#z = img[h]
	#z+=1
	#img[h] = z
	for j in range(len(imgName)):
		if h >= imgName[j][1] and h <= imgName[j][2]:
			imgName[j][3]+=1

print(imgName)