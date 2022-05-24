import cv2
import numpy as np

ref_img = cv2.imread('/home/bhertel/Downloads/screwdriver_sift.jpg')
test_img1 = cv2.imread('screwdriver.png')

rx, ry, _ = np.shape(ref_img)

#ref_img = cv2.resize(ref_img,(int(rx * 0.3), int(ry * 0.3)))

blur1 = cv2.GaussianBlur(ref_img,(5,5),cv2.BORDER_DEFAULT)
blur2 = cv2.GaussianBlur(test_img1,(5,5),cv2.BORDER_DEFAULT)

sift = cv2.xfeatures2d.SIFT_create()

keypoints1, des1= sift.detectAndCompute(blur1, None)
keypoints2, des2= sift.detectAndCompute(blur2, None)

bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

matches1 = bf.match(des1,des2)
matches1 = sorted(matches1, key= lambda match : match.distance)

keypoint_dists = []
for mat1 in matches1[:15]:
    dists = []
    for mat2 in matches1[:15]:
        dists.append(((keypoints2[mat1.trainIdx].pt[0] - keypoints2[mat2.trainIdx].pt[0])**2 + (keypoints2[mat1.trainIdx].pt[1] - keypoints2[mat2.trainIdx].pt[1])**2)**0.5)
        sort_dists = sorted(dists)
        dist = sum(sort_dists[:5])
        keypoint_dists.append(dist)
    best_key = np.argmin(keypoint_dists)


matched_img1 = cv2.drawMatches(blur1, keypoints1, blur2, keypoints2, matches1[:15], None)


start_point = keypoints2[matches1[best_key].trainIdx].pt
end_point = keypoints2[matches1[best_key].trainIdx].pt

start_point = (int(start_point[0] - 20), int(start_point[1] - 20))
end_point = (int(end_point[0] + 20), int(end_point[1] + 20))

cv2.rectangle(blur2, start_point, end_point, (0, 0, 255), thickness= 3, lineType=cv2.LINE_8)  

cv2.imshow("object", blur2)
cv2.imshow("matched", matched_img1)

cv2.waitKey(0)
cv2.destroyAllWindows()
