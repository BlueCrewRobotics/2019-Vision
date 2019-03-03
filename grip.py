import cv2
import numpy
import math
from enum import Enum

class VTapeOffsetDetection:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__rgb_threshold_red = [236.19604316546764, 255.0]
        self.__rgb_threshold_green = [236.19604316546764, 255.0]
        self.__rgb_threshold_blue = [234.55305755395685, 255.0]

        self.rgb_threshold_output = None

        self.__find_lines_input = self.rgb_threshold_output

        self.find_lines_output = None

        self.__find_contours_input = self.rgb_threshold_output
        self.__find_contours_external_only = False

        self.find_contours_output = None


    # returns the offset from center
    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step RGB_Threshold0:
        self.__rgb_threshold_input = source0
        (self.rgb_threshold_output) = self.__rgb_threshold(self.__rgb_threshold_input, self.__rgb_threshold_red, self.__rgb_threshold_green, self.__rgb_threshold_blue)
        
        # Runs these two different functions in parallel?

        # Step Find Lines:
        self.find_lines_output = self.__find_lines(self.rgb_threshold_output)
        
        # Step Find_Contours0:
        self.__find_contours_input = self.rgb_threshold_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        # find the angle of each line
        print("Agles of all lines")
        for x in self.find_lines_output:
            print(int(x.angle()))


        

        # finds the centers of each line
        xValues = []
        yValues = []
        for c in self.find_contours_output:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            xValues.append(cX)
            yValues.append(cY)
        imgHeight, imgWidth, imgChannels = source0.shape
        centerLine = imgWidth/2
        print("x data for each blob:")
        print(xValues)
        print("y data for each blob:")
        print(yValues)

        print("Center Line: ", centerLine)
        midPoint = int((xValues[0] + xValues[1]) / 2)
        print("Midpoint of tape:", midPoint)
        return midPoint - centerLine

    

    @staticmethod
    def __rgb_threshold(input, red, green, blue):
        """Segment an image based on color ranges.
        Args:
            input: A BGR numpy.ndarray.
            red: A list of two numbers the are the min and max red.
            green: A list of two numbers the are the min and max green.
            blue: A list of two numbers the are the min and max blue.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
        return cv2.inRange(out, (red[0], green[0], blue[0]),  (red[1], green[1], blue[1]))

    class Line:

        def __init__(self, x1, y1, x2, y2):
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2

        def length(self):
            return numpy.sqrt(pow(self.x2 - self.x1, 2) + pow(self.y2 - self.y1, 2))

        def angle(self):
            # return (math.degrees(math.atan2(self.y2 - self.y1, self.x2 - self.x1)))
            degree = math.degrees(math.atan2(self.y2 - self.y1, self.x2 - self.x1))
            if(degree > 90):
                return 180 - degree
            if(degree < -90):
                return -180 - degree
            return degree
            
    @staticmethod
    def __find_lines(input):
        """Finds all line segments in an image.
        Args:
            input: A numpy.ndarray.
        Returns:
            A filtered list of Lines.
        """
        detector = cv2.createLineSegmentDetector()
        lines = detector.detect(input)
        output = []
        
        if len(lines) != 0:
            for i in range(0, len(lines[0])):
                tmp = VTapeOffsetDetection.Line(lines[0][i, 0][0], lines[0][i, 0][1],
                                        lines[0][i, 0][2], lines[0][i, 0][3])
                output.append(tmp)
        return output

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
            method = cv2.CHAIN_APPROX_SIMPLE
            contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours



