import numpy as np
import cv2 as cv
import pickle
# Python program to show time by process_time()  
from time import process_time

# Start the stopwatch / counter  
t1_start = process_time()

file_des = open('p128_des_large.obj', 'rb')
des_t = pickle.load(file_des)

file_imgName = open('p128_imgName_large.obj', 'rb')
imgName = pickle.load(file_imgName)

sift = cv.xfeatures2d.SIFT_create()

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
#search_params = dict(checks=50)

#flann_index = cv.flann_Index

oriimg = cv.imread('../VDMSDataset/dataset/img/surface5.jpg')
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
t0_stop = process_time()
print("Loading index")
#flann = flann_index(des_t, index_params)

#flann.save('saved_index')

flann = cv.flann_Index
#fl = cv.flann
#file_savedIdx = cv.flann.open('saved_index', 'rb')
#file_savedIdx = cv.flann_Index(file_savedIdx, index_params)
#retval	= cv.flann_Index.load(	des_t, 'saved_index' )
#flann.load('saved_index',des_t)
test = cv.flann_Index.load(des_t, 'saved_index')

t1_stop = process_time()
print("Searching...")
idx, dist = flann.knnSearch(des2, 1, params={})
t2_stop = process_time()
print(len(idx))
print(len(dist))
#print(idx)

# l = idx//128
# print(np.amax(l))
# print(imgName[np.amax(l)])
t30_stop = process_time()

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
	# k = h//128
	# imgName[k][3]+=1
t3_stop = process_time()
#print(imgName)
# print(idx)
# print(l)

for i in range(len(imgName)):
	#print(idx[0][0])
	#h = idx[i,0]
	#print(h)
	#z = img[h]
	#z+=1
	#img[h] = z
	# for j in range(len(imgName)):
	if imgName[i][0] == "surface5.jpg":
		print(imgName[i])


# Stop the stopwatch / counter 
t4_stop = process_time()
print("Elapsed time to SIFT describe:", t0_stop-t1_start)
print("Elapsed time to 'index':", t1_stop-t0_stop)
print("Elapsed time to search:", t2_stop-t1_stop)
print("Elapsed time to divide idx and get max:", t30_stop-t2_stop)
print("Elapsed time to iterate idx:", t3_stop-t30_stop)
print("Elapsed time during the whole program in seconds:", t4_stop-t1_start)