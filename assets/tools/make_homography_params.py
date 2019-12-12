# MIT License
#
# Copyright (c) 2019 Atsushi Takada
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import cv2
import yaml
import time
import numpy as np
import math
import sys

class mouseParam:
    def __init__(self, input_img_name):

        self.mouseEvent = {"x":None, "y":None, "event":None, "flags":None}

        cv2.setMouseCallback(input_img_name, self.__CallBackFunc, None)


    def __CallBackFunc(self, eventType, x, y, flags, userdata):

        self.mouseEvent["x"] = x
        self.mouseEvent["y"] = y
        self.mouseEvent["event"] = eventType
        self.mouseEvent["flags"] = flags


    def getData(self):
        return self.mouseEvent


    def getEvent(self):
        return self.mouseEvent["event"]


    def getFlags(self):
        return self.mouseEvent["flags"]


    def getX(self):
        return self.mouseEvent["x"]


    def getY(self):
        return self.mouseEvent["y"]


    def getPos(self):
        return (self.mouseEvent["x"], self.mouseEvent["y"])

    def getPosNP(self):
        return [self.mouseEvent["x"], self.mouseEvent["y"]]


def set_dst_pt(img_witdh, img_height, pt_ratio, dst_pt_center_offset_x, dst_pt_center_offset_y):
    center_x = img_witdh/2
    center_y = img_height/2
    short_side_pix = img_witdh if img_height > img_witdh else img_height
    margin         = (short_side_pix * pt_ratio) / 2

    ox = dst_pt_center_offset_x
    oy = dst_pt_center_offset_y

    pt0_x = center_x - margin + ox
    pt0_y = center_y - margin + oy

    pt1_x = center_x - margin + ox
    pt1_y = center_y + margin + oy

    pt2_x = center_x + margin + ox
    pt2_y = center_y - margin + oy

    pt3_x = center_x + margin + ox
    pt3_y = center_y + margin + oy

    return np.float32([[pt0_x,pt0_y],[pt1_x,pt1_y],[pt2_x,pt2_y],[pt3_x,pt3_y]])



if __name__ == "__main__":
    print("enter filepath")
    filepath = raw_input(">>")
    src = cv2.imread(filepath)

    dst = np.zeros_like(src)

    src_width  = np.size(src, 1)
    src_height = np.size(src, 0)
    select_pt = []
    cv2.imshow("src", src)
    mouseData = mouseParam("src")

    src_pt_set_flag = False
    while src_pt_set_flag == False:
        cv2.imshow("src", src)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break

        if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(src, mouseData.getPos() , 2, (0, 0, 255), thickness=-1, lineType=cv2.LINE_AA)
            time.sleep(0.2)
            select_pt.append(mouseData.getPos())
            print(mouseData.getPos())
            if len(select_pt) == 4:
                src_pt_set_flag = True

    src_pt = np.float32([[select_pt[0][0],select_pt[0][1]],
                         [select_pt[1][0],select_pt[1][1]],
                         [select_pt[2][0],select_pt[2][1]],
                         [select_pt[3][0],select_pt[3][1]]])


    src_pt_set_flag = False
    dst_pt = set_dst_pt(src_width, src_height, 1.0, 0, 0)
    M = cv2.getPerspectiveTransform(src_pt, dst_pt)
    dst = cv2.warpPerspective(src, M, (src_width, src_height))
    cv2.imshow("dst", dst)

    while src_pt_set_flag == False:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            src_pt_set_flag = True
        elif cv2.waitKey(1) & 0xFF == ord("c"):
            print("please input param")
            ratio = input("ratio: ")
            offset_x = ((select_pt[0][0] + select_pt[2][0]) / 2) - (src_width / 2)
            offset_y = input("offset_y: ")
            dst_pt = set_dst_pt(src_width, src_height, ratio, offset_x, offset_y)
            M = cv2.getPerspectiveTransform(src_pt, dst_pt)
            dst = cv2.warpPerspective(src, M, (src_width,src_height))
            cv2.imshow("dst", dst)


inverse_M = np.linalg.inv(M)
print("matrix")
print(inverse_M)

print("matrix for YAML")
print(inverse_M[0][0], inverse_M[0][1], inverse_M[0][2],
      inverse_M[1][0], inverse_M[1][1], inverse_M[1][2],
      inverse_M[2][0], inverse_M[2][1], inverse_M[2][2])
