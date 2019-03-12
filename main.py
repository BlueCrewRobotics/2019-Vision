from networktables import NetworkTables 
#from grip import VTapeOffsetDetection
import cv2


print("beginning of vision code")
NetworkTables.initialize(server="roborio-6153-frc.local")
sd = NetworkTables.getTable("SmartDashboard")

while(True):
    sd.putNumber("testNumner", 5)
'''
image = cv2.imread("photo.jpg")
offset = VTapeOffsetDetection()
print(offset.process(image))
'''
