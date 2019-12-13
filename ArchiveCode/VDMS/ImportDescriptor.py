#import csv
#import vdms
#import util
#import os
import numpy as np
import cv2 as cv

oriimg = cv.imread('VDMSDataset/dataset/img/surface1.jpg')
print(oriimg.shape)
img = cv.resize(oriimg,None,fx=0.5,fy=0.5)
print(img.shape)
imgrgb= cv.cvtColor(img,cv.COLOR_BGR2RGB)
sift = cv.xfeatures2d.SIFT_create()
print("keypoints starting")
kp = sift.detect(imgrgb,None)
print("keypoints done")
kp,des = sift.compute(imgrgb,kp)
#kp, des = sift.detectAndCompute(imgrgb,None)
print("Compute done")
#db = vdms.vdms()
#db.connect("localhost") # Will connect to localhost on port 55555

# Insert a DescriptorSet for doing face matching
query = """
[
   {
	    "AddDescriptorSet": {
	        "engine": "FaissFlat",
	        "metric": "L2",
	        "name": "test_ds",
	        "dimensions": 128
	    }
	}
]   
"""

query2 = """
[
	{
      "FindImage": {
      		"_ref": 344554,
		    "constraints" : {
		        "date": [ "==", "October 24rd, 2019" ]
		    },
		   "results": {
		        "list": [ "name_file", "type", "date" ]
		    }
		}
   },
   {
	    "AddDescriptor": {
	        "set": "test_ds",
	        "label": "Tea Test",
	        "link": {                   // We can create a connection between the image
		        "ref": 344554                // and the descriptor.
		    }
	    }
	}
]   
"""
print("query defined")

blob_array = []

for ele in des:
	blob_array.append(np.array(ele).astype('float32').tobytes())
print("blob created")
#print(blob_array)


#response, images = db.query(query)
#response, res_arr = db.query(query2, [blob_array])
#print (response)
#print (db.get_last_response_str())