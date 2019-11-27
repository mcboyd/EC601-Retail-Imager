#!C:/Python27
#coding=utf-8
 
import pytesseract
from PIL import Image,ImageEnhance,ImageFilter
import os
import fnmatch
import re,time
 
import urllib, random
 
 
#import hashlib  
 
def getGray(image_file):
   tmpls=[]
   for h in range(0,  image_file.size[1]):#h
      for w in range(0, image_file.size[0]):#w
         tmpls.append( image_file.getpixel((w,h))  )
          
   return tmpls
 
def getAvg(ls):#get average gray value
   return sum(ls)/len(ls)
 
def getMH(a,b):#compare the same value
   dist = 0;
   for i in range(0,len(a)):
      if a[i]==b[i]:
         dist=dist+1
   return dist
 
def getImgHash(fne):
   image_file = Image.open(fne)
   image_file=image_file.resize((12, 12))#resize the image
   image_file=image_file.convert("L")#256 grey value img
   Grayls=getGray(image_file)#get all grey value 
   avg=getAvg(Grayls)#average value
   bitls=''#
   for h in range(1,  image_file.size[1]-1):#h
      for w in range(1, image_file.size[0]-1):#w
         if image_file.getpixel((w,h))>=avg:
            bitls=bitls+'1'
         else:
            bitls=bitls+'0'
   return bitls
 
#img address
a=getImgHash("/home/elvis/Downloads/box/box1.jpg")
b=getImgHash("/home/elvis/Downloads/box/box1.jpg")
compare=getMH(a,b)
print (str(compare)+'%')    