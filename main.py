from networktables import NetworkTables 
from grip import VTapeOffsetDetection
import cv2

image = cv2.imread("photo.jpg")
offset = VTapeOffsetDetection()
print(offset.process(image))
