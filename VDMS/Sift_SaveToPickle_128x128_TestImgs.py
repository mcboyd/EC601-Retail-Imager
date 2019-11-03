import numpy as np
import cv2 as cv
import pickle
import os

dirName = '../VDMSDataset/images/';

# Get the list of all files in directory tree at given path
listOfFiles = list()
for (dirpath, dirnames, filenames) in os.walk(dirName):
    listOfFiles += [os.path.join(dirpath, file) for file in filenames]

sift = cv.xfeatures2d.SIFT_create()

imgName=[]
des_t = None
des_i = -1
zerow = np.zeros((1, 128),dtype="float32")

for x in listOfFiles:
	oriimg = cv.imread(x)
	#print('../VDMSDataset/dataset/img/'+x)
	#img = cv.resize(oriimg,None,fx=0.2,fy=0.2)
	#print(img.shape)
	imgrgb= cv.cvtColor(oriimg,cv.COLOR_BGR2RGB)
	#print("keypoints starting for {}", x)
	kp = sift.detect(imgrgb,None)
	kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value(bigger is better)
	#print("keypoints done for {}, keypoints: {}", x, len(kps))
	kps,des = sift.compute(imgrgb,kps)
	
	#print(des)
	if x == 'surface10.jpg':
		des2 = des
	else:
		#des_all.append(des[0])
		#print(des.shape)
		#np.savetxt(x+'_orig.out', des)

		#print(zerow)
		if len(des)< 128:
			print(x)
			#print(des.shape)
		while len(des)<128:
			#print(des.shape)
			des = np.concatenate([des, zerow])
		#print(des.shape)
		#print(des)
		#np.savetxt(x+'.out', des)

		imgName.append([x,des_i+1,des_i+len(des),0])
		des_i+=len(des)
		
		if des_t is None:
			des_t = des
		else:
			des_t = np.concatenate((des_t,des))

files = ["surface1.jpg","surface2.jpg","surface3.jpg","surface4.jpg","surface5.jpg","surface6.jpg","surface7.jpg","surface8.jpg","surface9.jpg",]
for x in files:
	oriimg = cv.imread('../VDMSDataset/dataset/img/'+x)
	#print('../VDMSDataset/dataset/img/'+x)
	img = cv.resize(oriimg,None,fx=0.2,fy=0.2)
	#print(img.shape)
	imgrgb= cv.cvtColor(img,cv.COLOR_BGR2RGB)
	print("keypoints starting for {}", x)
	kp = sift.detect(imgrgb,None)
	kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value(bigger is better)
	print("keypoints done for {}", x)
	kps,des = sift.compute(imgrgb,kps)
	
	#print(des)
	if x == 'surface10.jpg':
		des2 = des
	else:
		#des_all.append(des[0])
		#print(des.shape)
		imgName.append([x,des_i+1,des_i+len(des),0])
		des_i+=len(des)
		if des_t is None:
			des_t = des
		else:
			des_t = np.concatenate((des_t,des))

object_des = des_t
file_des = open('p128_des_large.obj', 'wb')
pickle.dump(object_des, file_des)
object_imgName = imgName
file_imgName = open('p128_imgName_large.obj', 'wb')
pickle.dump(object_imgName, file_imgName)



