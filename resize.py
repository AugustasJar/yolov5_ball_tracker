#Resize images to 640x640

import cv2

img = cv2.imread('img_1.jpg', cv2.IMREAD_UNCHANGED)

print('Original Dimensions : ', img.shape)

width = 640  # keep original width
height = 640
dim = (width, height)

# resize image
resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

print('Resized Dimensions : ', resized.shape)

cv2.imshow("Resized image", resized)
cv2.imwrite("Resized.jpg", resized)
cv2.waitKey(0)
cv2.destroyAllWindows()

