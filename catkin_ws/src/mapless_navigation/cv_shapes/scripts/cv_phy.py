#!/usr/bin/env python
import cv2

print(cv2.samples.findFile("building.jpg"))
image = cv2.imread('building.jpg')

cv2.imshow("Image", image)
cv2.waitKey(0)
# Closing all windows
cv2.destroyAllWindows()



