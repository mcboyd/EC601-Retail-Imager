# Resize images and load features and descriptors into one large array
# Add array to FLANN training
# Search for one of the images in array

import numpy as np
import cv2 as cv

files = ["surface1.jpg","surface2.jpg","surface3.jpg","surface4.jpg","surface5.jpg","surface6.jpg","surface7.jpg","surface8.jpg","surface9.jpg",]

sift = cv.xfeatures2d.SIFT_create()

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

flann_index = cv.flann_Index
#flann = cv.FlannBasedMatcher(index_params, search_params)

des_all = []
imgName=[]
des_t = None
des_i = -1

for x in files:
	oriimg = cv.imread('../VDMSDataset/dataset/img/'+x)
	#print('../VDMSDataset/dataset/img/'+x)
	img = cv.resize(oriimg,None,fx=0.2,fy=0.2)
	#print(img.shape)
	imgrgb= cv.cvtColor(img,cv.COLOR_BGR2RGB)
	print("keypoints starting for {}", x)
	kps,des = sift.detectAndCompute(imgrgb,None)
	#print(des)
	if x == 'surface2.jpg':
		des2 = des
	else:
		#des_all.append(des[0])
		print(des.shape)
		imgName.append([x,des_i+1,des_i+len(des),0])
		des_i+=len(des)
		if des_t is None:
			des_t = des
		else:
			des_t = np.concatenate((des_t,des))


print("keypoints done")
print(len(des_t))
print(des_t.shape)
#print(des_all)
# print(des.shape)
print(des2.shape)

#des_np = np.array([des_all[0],des_all[1],des_all[2],des_all[3],des_all[4],des_all[5],des_all[6],des_all[7],des_all[8]],np.float32)
#des_all = np.array(des_all)
#print(des_all.shape)

print("Building index")
flann = flann_index(des_t, index_params)
print("Searching...")
idx, dist = flann.knnSearch(des2, 1, params={})

print(len(idx))
print(len(dist))
# print(idx)
# print(dist)
print(idx[0])

# print(indexList)

# votes = indexList[idx]
# from collections import Counter
# votes = Counter()
# likelyMatch = votes.most_common()[0][0]
# print(likelyMatch)

#img=[0,0,0,0,0,0,0,0]

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
#print(img)

#flann.add(des_all)
#print("Training...")
#flann.train()
#print("Matching...")
#indexes, matches = flann.knnSearch(des, 2)
#matches = flann.knnSearch(queryDescriptors=des, k=2)

#print(matches.trainIdx)