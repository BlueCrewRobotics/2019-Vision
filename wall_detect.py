from networktables import NetworkTables 
import cv2
import numpy
import math
from enum import Enum

class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_hue = [68.0, 90]
        self.__hsl_threshold_saturation = [126, 255.0]
        self.__hsl_threshold_luminance = [83, 255]

        self.hsl_threshold_output = None


        self.__mask_mask = self.hsl_threshold_output

        self.mask_output = None

        self.__find_blobs_input = self.mask_output
        self.__find_blobs_min_area = 20.0
        self.__find_blobs_circularity = [0.0, 1.0]
        self.__find_blobs_dark_blobs = False

        self.find_blobs_output = None


    def process(self, source0, width):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        
        centerLine = width/2

        # Step HSL_Threshold0:
        self.__hsl_threshold_input = source0
        (self.hsl_threshold_output) = self.__hsl_threshold(self.__hsl_threshold_input, self.__hsl_threshold_hue, self.__hsl_threshold_saturation, self.__hsl_threshold_luminance)

        # Step Mask0:
        self.__mask_input = source0
        self.__mask_mask = self.hsl_threshold_output
        (self.mask_output) = self.__mask(self.__mask_input, self.__mask_mask)

        # Step Find_Blobs0:
        self.__find_blobs_input = self.mask_output
        (self.find_blobs_output) = self.__find_blobs(self.__find_blobs_input, self.__find_blobs_min_area, self.__find_blobs_circularity, self.__find_blobs_dark_blobs)

        xValues = []
        for blob in self.find_blobs_output:
            xValues.append(blob.pt[0])
        print(xValues)
        # if it only detects 1 then do nothing 
        if(len(xValues) < 2):
            return 0

        return centerLine - ((xValues[0] + xValues[1]) / 2)


    @staticmethod
    def __hsl_threshold(input, hue, sat, lum):
        """Segment an image based on hue, saturation, and luminance ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max luminance.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
        return cv2.inRange(out, (hue[0], lum[0], sat[0]),  (hue[1], lum[1], sat[1]))

    @staticmethod
    def __mask(input, mask):
        """Filter out an area of an image using a binary mask.
        Args:
            input: A three channel numpy.ndarray.
            mask: A black and white numpy.ndarray.
        Returns:
            A three channel numpy.ndarray.
        """
        return cv2.bitwise_and(input, input, mask=mask)

    @staticmethod
    def __find_blobs(input, min_area, circularity, dark_blobs):
        """Detects groups of pixels in an image.
        Args:
            input: A numpy.ndarray.
            min_area: The minimum blob size to be found.
            circularity: The min and max circularity as a list of two numbers.
            dark_blobs: A boolean. If true looks for black. Otherwise it looks for white.
        Returns:
            A list of KeyPoint.
        """
        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = 1
        params.blobColor = (0 if dark_blobs else 255)
        params.minThreshold = 10
        params.maxThreshold = 220
        params.filterByArea = True
        params.minArea = min_area
        params.filterByCircularity = True
        params.minCircularity = circularity[0]
        params.maxCircularity = circularity[1]
        params.filterByConvexity = False
        params.filterByInertia = False
        detector = cv2.SimpleBlobDetector_create(params)
        return detector.detect(input)


if (__name__=="__main__"):

    print("beginning of vision code")
    NetworkTables.initialize(server="roborio-6153-frc.local")
    sd = NetworkTables.getTable("SmartDashboard")
    cap = cv2.VideoCapture(0)
    pipeline = GripPipeline()

    while(cap.isOpened()):
        have_frame, frame = cap.read()
        imgHeight, imgWidth, imgChannels = frame.shape 
        offset = pipeline.process(frame, 120)
        print(offset)
        sd.putNumber("VTape_Offset", offset)
        sd.putNumber("cameraWidth", imgWidth)
 
