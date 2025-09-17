import cv2
import numpy as np
import camtools


image = cv2.imread('car_view.png')
cv2.imshow("before",image)

gray_image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow("GreyScale",gray_image)
gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
cv2.imshow("GreyScadle",gray_image)

lines = cv2.Canny(gray_image,40,200)

mask = np.zeros(lines.shape[:2], dtype="uint8")
x, y, w, h = 0, 235, 784, 159  
cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1) 
masked_image = cv2.bitwise_and(lines, lines, mask=mask)


cv2.imshow("masked", masked_image)

cv2.waitKey(50000)
cv2.destroyAllWindows()
cv2.waitKey(1)