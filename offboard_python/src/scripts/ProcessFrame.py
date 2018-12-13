import cv2
import numpy as np
import math


class ProcessFrame(object):

    def __init__(self):
        self.average = [0, 0, 0, 0]
        self.resized_image = np.ndarray
        self.im_bw = np.ndarray
        self.lines = np.ndarray

    def run(self, frame, show=False):
        try:
            self.resized_image = frame
            # self.resized_image = np.rot90(frame)
            self.resized_image = cv2.resize(self.resized_image, (400, 800))
            self.im_bw = self.create_binary_image(self.resized_image, 'vertical')

            self.lines = self.apply_hough_transformP(self.im_bw)
            lines = self.find_best_lines(self.lines, 10, 10, self.resized_image)
            self.print_probabilistic_lines(self.resized_image, lines)
            self.print_average_line(self.resized_image, lines)

            #if show:
                #cv2.imshow('res', self.resized_image)
                #cv2.imshow('canny', self.im_bw)
            return self.yaw_angle()
        except ValueError:
            pass

    def create_binary_image(self, original_image, filter='vertical'):
        img_greyscale = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

        # Convolving the Grey-scale image with a kernel of 2 by 20px.
        if filter == 'horizontal':
            kernel = np.matrix([[-1, -1, -1],
                                [2, 2, 2],
                                [-1, -1, -1]])
            img_lines = cv2.filter2D(img_greyscale, -1, kernel)
            horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 1))
            img_line1 = cv2.erode(img_lines, horizontal_structure, iterations=1)
            img_lin2 = cv2.dilate(img_line1, horizontal_structure, iterations=1)

        else:

            kernel = np.matrix([[-1, 2, -1]])
            img_lines = cv2.filter2D(img_greyscale, -1, kernel)
            vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 20))
            img_line1 = cv2.erode(img_lines, vertical_structure, iterations=1)
            img_lin2 = cv2.dilate(img_line1, vertical_structure, iterations=1)

        (thresh, im_bw) = cv2.threshold(img_lin2, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        return im_bw

    def apply_hough_transformP(self, binary_image):
        return cv2.HoughLinesP(binary_image, rho=1, theta=np.pi / 180, threshold=60, minLineLength=150, maxLineGap=200)

    def print_average_line(self, original_image, lines):

        if lines.size != 0:
            points = np.mean(lines, axis=1).astype(int)
            height, width, channels = original_image.shape
            x1 = points[0]
            y1 = points[1]
            x2 = points[2]
            y2 = points[3]

            theta = np.arctan2(y2 - y1, x2 - x1)
            x1 = int(x1 - width * np.cos(theta))
            y1 = int(y1 - height * np.sin(theta))
            x2 = int(x2 + width * np.cos(theta))
            y2 = int(y2 + height * np.sin(theta))

            self.average[0] = (self.average[0] + x1) / 2
            self.average[1] = (self.average[1] + y1) / 2
            self.average[2] = (self.average[2] + x2) / 2
            self.average[3] = (self.average[3] + y2) / 2

            cv2.line(original_image, (self.average[0], self.average[1]), (self.average[2], self.average[3]),
                     (0, 255, 255), 2)

    def print_probabilistic_lines(self, orignal_image, lines):
        try:
            for i in range(0, len(lines[0])):
                cv2.line(orignal_image, (lines[0][i], lines[1][i]), (lines[2][i], lines[3][i]), (255, 0, 0), 2)
        except TypeError:
            pass

    def find_best_lines(self, lines, grouping, number_of_lines, original_image):
        height, width, channels = original_image.shape
        votes = np.zeros(width)
        x1_points = []
        x2_points = []
        y1_points = []
        y2_points = []

        try:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    votes[x1] += 1
        except TypeError:
            pass

        grouped_votes = np.add.reduceat(votes, np.arange(0, len(votes), grouping))
        indexes = np.argpartition(grouped_votes, -number_of_lines)[-number_of_lines:]
        try:
            for index in indexes:
                if grouped_votes[index]:
                    for line in lines:
                        for x1, y1, x2, y2 in line:
                            if index * grouping <= x1 < index * grouping + grouping:
                                x1_points.append(x1)
                                x2_points.append(x2)
                                y1_points.append(y1)
                                y2_points.append(y2)
        except TypeError:
            pass

        return np.asarray([x1_points, y1_points, x2_points, y2_points])

    def yaw_angle(self):
        x0 = self.average[0]
        y0 = self.average[1]
        x1 = self.average[2]
        y1 = self.average[3]
        print x0, y0, x1, y1
        if x0 == x1:
            distance = 0
            angle = 0
        elif x1 > x0 and y1 > y0:
            anglerad = math.atan(float(x1 - x0) / float(y0 - y1))
            angle = math.degrees(anglerad)
            distance = x1 - x0
        elif x1 < x0 and y1 > y0:
            anglerad = math.atan(float(x0 - x1) / float(y0 - y1))
            angle = math.degrees(anglerad)
            distance = x0 - x1
        elif y1 == y0 and x1 > x0:
            angle = 90
            distance = 0
        else:
            angle = None
            distance = None
        return distance, angle

    def apply_hough_transform(self, original_image, binary_image):
        lines = cv2.HoughLines(binary_image, 1, np.pi / 180, 100)
        number_of_lines = 1
        print lines
        try:
            for line in lines[:number_of_lines]:
                for rho, theta in line:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    cv2.line(original_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        except TypeError:
            pass
