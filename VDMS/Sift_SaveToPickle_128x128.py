import numpy as np
import cv2 as cv
import pickle

files = ["surface1.jpg","surface2.jpg","surface3.jpg","surface4.jpg","surface5.jpg","surface6.jpg","surface7.jpg","surface8.jpg","surface9.jpg",]

sift = cv.xfeatures2d.SIFT_create()

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
	kp = sift.detect(imgrgb,None)
	kps = sorted(kp, key=lambda x: -x.response)[:128]  # Grab only "top" 128 keypoints by response value(bigger is better)
	print("keypoints done for {}", x)
	kps,des = sift.compute(imgrgb,kps)
	
	#print(des)
	if x == 'surface10.jpg':
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

object_des = des_t
file_des = open('p128_des.obj', 'wb')
pickle.dump(object_des, file_des)
object_imgName = imgName
file_imgName = open('p128_imgName.obj', 'wb')
pickle.dump(object_imgName, file_imgName)



