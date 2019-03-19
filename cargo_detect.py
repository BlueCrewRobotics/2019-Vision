#!/usr/bin/python3

#
# Vision Processing Code
# Code to detect Power Cubes on the Nvidia Jetson TX1
# (c) 2018 Blue Crew Robotics
# Author: Matthew Gallant
#

import cv2
import numpy
import math
from enum import Enum
from networktables import NetworkTables

class GripPipeline():
    
    def __init__(self):

        self.__hsv_threshold_hue = [0.0, 63.87096774193549]
        self.__hsv_threshold_saturation = [57.32913669064748, 207.37691001697794]
        self.__hsv_threshold_value = [213.03507194244605, 255.0]

        self.hsv_threshold_output = None

        self.__blur_input = self.hsv_threshold_output
        self.__blur_type = BlurType.Median_Filter
        self.__blur_radius = 100.0

        self.blur_output = None

        self.__find_contours_input = self.blur_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 50000.0
        self.__filter_contours_min_perimeter = 0.0
        self.__filter_contours_min_width = 0.0
        self.__filter_contours_max_width = 1000.0
        self.__filter_contours_min_height = 0.0
        self.__filter_contours_max_height = 1000.0
        self.__filter_contours_solidity = [0, 100]
        self.__filter_contours_max_vertices = 1000000.0
        self.__filter_contours_min_vertices = 0.0
        self.__filter_contours_min_ratio = 0.0
        self.__filter_contours_max_ratio = 1000.0

        self.filter_contours_output = None


    def process(self, source0):
        # Step HSV_Threshold0:
        self.__hsv_threshold_input = source0
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

        # Step Blur0:
        self.__blur_input = self.hsv_threshold_output
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type, self.__blur_radius)

        # Step Find_Contours0:
        self.__find_contours_input = self.blur_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours, self.__filter_contours_min_area, self.__filter_contours_min_perimeter, self.__filter_contours_min_width, self.__filter_contours_max_width, self.__filter_contours_min_height, self.__filter_contours_max_height, self.__filter_contours_solidity, self.__filter_contours_max_vertices, self.__filter_contours_min_vertices, self.__filter_contours_min_ratio, self.__filter_contours_max_ratio)


    @staticmethod
    def __hsv_threshold(input, hue, sat, val):
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    # if we determine the BlurType can we get rid of this whole if tree? 
    # or maybe prioritize the default one to the top?
    # the same can be said about a lot of the other things
    @staticmethod
    def __blur(src, type, radius):
        if(type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif(type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif(type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __find_contours(input, external_only):
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                        min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                        min_ratio, max_ratio):
        output = []
        for contour in input_contours:

            x,y,w,h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue

            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue

            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                continue
            
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')



def extra_processing(pipeline, sd):
    center_x_positions = []
    center_y_positions = []
    widths = []
    heights = []

    # Find the bounding boxes of the contours to get x, y, width, and height
    for contour in pipeline.filter_contours_output:
        x, y, w, h = cv2.boundingRect(contour)
        center_x_positions.append(x + w / 2)  # X and Y are coordinates of the top-left corner of the bounding box
        center_y_positions.append(y + h / 2)
        widths.append(w)
        heights.append(h)
    
    # Print output
    print("center X: ", center_x_positions)
    print("center Y: ", center_y_positions)
    print("width: ", widths)
    print("height: ", heights)

    sd.putValue("Center X", str(center_x_positions))
    sd.putValue("Center Y", str(center_y_positions))
    sd.putValue("Width", str(widths))
    sd.putValue("Height", str(heights))

def main():
    NetworkTables.initialize(server='roborio-6153-frc.local')
    sd = NetworkTables.getTable("SmartDashboard")
    
    cap = cv2.VideoCapture(0)
    pipeline = GripPipeline()

    while cap.isOpened():
        have_frame, frame = cap.read()
        if have_frame:
            new_frame = cv2.resize(frame, (285, 160))
            # should we be processing new_frame?
            pipeline.process(frame)
            extra_processing(pipeline, sd)

if __name__ == '__main__':
    main()
