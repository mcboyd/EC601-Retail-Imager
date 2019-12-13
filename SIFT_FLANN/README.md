# EC601 Retail Product Imager

#### SIFT_FLANN Folder

Python code to build FLANN indexes (using sample retail images and production images), extract image features using SIFT, and search FLANN indexes for an image match. Five files:

- FLANN_Index_GenerateSave.py: *Builds a FLANN index file (used in image matching) and saves it to disk. Loads 2 previously created files from disk: a blob object of SIFT feature descriptions and a text object of image descrptions; generates the index and saves it as a new blob object to disk.*
- FLANN_Index_Search.py: *Searches for an image match in a FLANN index. Loads 2 previously created files from disk: a blob object FLANN index file (created with FLANN_Index_GenerateSave.py) and a text object of image descrptions; opens the image name indicated (as an input argument); SIFT describes the image; loads the index into memory; searches for the closest image match in the index; returns the original image file name and the "product id" (from our database and specified when importing the images using SIFT_ImportImages_SaveObj.py) of the closest image match in the index.*
- imgextract.py: *Helper file that extracts the product image from the image of the whole scene. Takes 4 corner points as input.*
- imgextract_prod.py: *Front-end file that calls "imagextract.py" to extract the product image based on the input corner points.*
- SIFT_ImportImages_SaveObj.py: *Imports images to build FLANN indexes. Only designed to run occasionally when building indexes, so you must edit the image paths and file names directly inside this file. Imports each image, SIFT describes it, filters down to the top 128 features, adds the features to a matrix, and adds text descrptors (original file name, product ID, and start and end index positions) to text array. Exports final feature matrix as a blob file to disk and text array as a text file to disk.*

## How to use

Python (we used 3.7.5) and OpenCV (with "contrib" and "nonfree" algorithms enabled - we used 3.4.7) must be installed and for your platform (Windows or Linux). Other dependencies indicated in the individual files also required, but they're much easier to install as usual. 


