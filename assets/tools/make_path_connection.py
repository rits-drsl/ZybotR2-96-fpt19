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

# python2
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


def yaml_point_read(path, index):
        return (path["point"+ str(index).zfill(3)]["x"], path["point"+ str(index).zfill(3)]["y"])

def yaml_num_read(path):
        return path["num"]

def get_norm(point1,point2):
        absx = abs(point1[0] - point2[0])
        absy = abs(point1[1] - point2[1])
        return math.sqrt(absx*absx + absy*absy)


if __name__ == "__main__":
    save_filepath = "../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/"

    #print("enter img_filepath(.png)")
    # img_filepath = raw_input(">>")
    # read = cv2.imread(save_filepath + img_filepath)
    read = cv2.imread(save_filepath + "line.png")

    window_name = "input window"
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

    # read paths yaml
    print("enter paths_filepath(.yaml)")
    paths_filepath = raw_input(">>")
    fs = open(save_filepath + paths_filepath, "r")
    # fs = open("../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/paths1.yaml", "r")

    paths_map = yaml.load(fs)
    fs.close()

    paths = []
    for i in range(paths_map["num"]):
        path = []
        for j in range(paths_map["path" + str(i)]["num"]):
            point = yaml_point_read(paths_map["path" + str(i)], j)
            one_based_index = i + 1
            cv2.circle(read, point, 4, ((one_based_index & 0b001) * 255, (one_based_index & 0b010) * 255, (one_based_index & 0b100) * 255), -1)
            path.append(point)
        paths.append(path)

    src_path_number = []
    src_index = []
    dst_path_number = []
    dst_index = []

    window_name = "input window"
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

    mouseData = mouseParam(window_name)

    write_data = {}

    cv2.putText(read, "quit: q", (300,  50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "save: s", (300,  80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "src_point : leftclick" , (20,  200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "dst_point : rightclick", (20,  250), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    while 1:
        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit()

        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break

        if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN:
            time.sleep(0.2)
            min_norm = 1000000000
            min_index = -1
            min_number = -1
            for ni in range(5):
                for ii in range(len(paths[ni])):
                    norm = get_norm(paths[ni][ii],mouseData.getPos())
                    if norm < min_norm:
                        min_norm   = norm
                        min_index  = ii
                        min_number = ni
            cv2.drawMarker(read, paths[min_number][min_index] ,(240, 240, 240), markerType=cv2.MARKER_STAR, markerSize=15)
            src_path_number.append(min_number)
            src_index.append(min_index)

        elif mouseData.getEvent() == cv2.EVENT_RBUTTONDOWN:
            time.sleep(0.2)
            min_norm2 = 1000000000
            min_index2 = -1
            min_number2 = -1
            for ni in range(5):
                for ii in range(len(paths[ni])):
                    norm2 = get_norm(paths[ni][ii],mouseData.getPos())
                    if norm2 < min_norm2:
                        min_norm2   = norm2
                        min_index2  = ii
                        min_number2 = ni
            dst_path_number.append(min_number2)
            dst_index.append(min_index2)
            cv2.drawMarker(read, paths[min_number2][min_index2] ,(240, 240, 240), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15)
            cv2.arrowedLine(read, paths[min_number][min_index], paths[min_number2][min_index2], (0, 255, 255), thickness=2)


    cv2.destroyAllWindows()
    print("enter path_connection_filepath(.yaml)")
    path_connection_filename = raw_input(">>")
    f = open(save_filepath + path_connection_filename, "w")
    # f = open("../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/path_connection4.yaml", "w")

    f.write("%YAML 1.2\n")
    f.write("---\n")

    if (len(src_path_number) == len(dst_path_number)):
        conn_info_array = {}
        for i in range(len(src_path_number)):
            conn_info_array["conn"+str(i).zfill(2)] =  {"src_path_number": src_path_number[i], "src_path_index": src_index[i], "dst_path_number": dst_path_number[i], "dst_path_index": dst_index[i]}
        write_data = conn_info_array
        f.write(yaml.dump(write_data, default_flow_style=False))
    else:
        print("please again")

    f.write("num: " + str(len(src_path_number)))
    f.close()
