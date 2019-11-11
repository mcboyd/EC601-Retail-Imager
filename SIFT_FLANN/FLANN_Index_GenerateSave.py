import numpy as np
import cv2 as cv
import pickle
import sys

if len(sys.argv)<=1:
	sys.exit("Requires 1 parameter: sift obj name to generate index for")

indexFile = sys.argv[1]  # test with 'large'

file_des = open('p128_des_'+indexFile+'.obj', 'rb')  # SIFT obj of feature descriptions
des_t = pickle.load(file_des)

file_imgName = open('p128_imgName_'+indexFile+'.obj', 'rb')  # Index description of images in des
imgName = pickle.load(file_imgName)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

flann_index = cv.flann_Index

# Build index
flann = flann_index(des_t, index_params)

# Save the generated index to disk (for retrieval and loading above)
flann.save('p128_des_'+indexFile+'_idx.obj')

print("complete: " + indexFile)