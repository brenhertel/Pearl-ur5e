import cv2
import numpy as np

ref_img = cv2.imread('/home/bhertel/Downloads/1_sub_2.png')
test_img1 = cv2.imread('/home/bhertel/Downloads/slide1.png')
test_img2 = cv2.imread('/home/bhertel/Downloads/slide2.png')

print(test_img1)

cv2.imshow('img', test_img1)
cv2.waitKey(1000)
cv2.destroyAllWindows()

#blur1 = cv2.GaussianBlur(r_resize,(5,5),cv2.BORDER_DEFAULT)
#blur2 = cv2.GaussianBlur(f_resize,(5,5),cv2.BORDER_DEFAULT)
#
#sift = cv2.xfeatures2d.SIFT_create()
#
#keypoints1, des1= sift.detectAndCompute(blur1, None)
#keypoints2, des2= sift.detectAndCompute(blur2, None)
#
#bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
#
#matches1 = bf.match(des1,des2)
#matches1 = sorted(matches1, key= lambda match : match.distance)
#matched_img1 = cv2.drawMatches(blur1, keypoints1, blur2, keypoints2, matches1[:30], None)
#
#cv2.imshow("Match Attempt 1", matched_img1)
#cv2.waitKey(1000)
#cv2.destroyAllWindows()
