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
from collections import OrderedDict

def ordered_load(stream, Loader=yaml.Loader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass
    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)
    return yaml.load(stream, OrderedLoader)

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
    # read = cv2.imread(save_filepath+ img_filepath)
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

    # about velocity
    velocity = {"velocity": {"straight"     : 0.1,
                             "curve"        : 0.03,
                             "junction"     : 0.05}}
    # set velocity
    s = '"'
    velocity_straight     = {"velocity": s + "straight" + s}
    velocity_curve        = {"velocity": s + "curve" + s}
    velocity_junction     = {"velocity": s + "junction" + s}

    num_of_velocity_type = len(velocity["velocity"])
    velocity_mode = 0
    str_velocity_mode= str(velocity_mode)

    paths = []
    for i in range(paths_map["num"]):
        path = []
        for j in range(paths_map["path" + str(i)]["num"]):
            point = yaml_point_read(paths_map["path" + str(i)], j)
            one_based_index = i + 1
            cv2.circle(read, point, 4, ((one_based_index & 0b001) * 255, (one_based_index & 0b010) * 255, (one_based_index & 0b100) * 255), -1)
            path.append(point)
        paths.append(path)

    s = '"'

    # all straight
    for i in range(paths_map["num"]):
        for j in range(paths_map["path" + str(i)]["num"]):
            paths_map["path"+ str(i)]["point"+ str(j).zfill(3)].update(velocity_straight)

    window_name = "input window"
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

    mouseData = mouseParam(window_name)

    cv2.putText(read, "quit: q", (300,  50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "save: s", (300,  80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "change mode: c", (300,  110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "set velocity : leftclick" , (20,  200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "mode: "+str_velocity_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    # set text
    cv2.putText(read, "mode0: straight" , (800,  50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "mode1: curve", (800,  80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "mode2: junction", (800, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    while 1:
        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit()

        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break

        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("c"):
            if (velocity_mode < num_of_velocity_type - 1):
                cv2.putText(read, "mode: "+str_velocity_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                velocity_mode += 1
                str_velocity_mode = str(velocity_mode)
                cv2.putText(read, "mode: "+str_velocity_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
            else:
                cv2.putText(read, "mode: "+str_velocity_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                velocity_mode = 0
                str_velocity_mode = str(velocity_mode)
                cv2.putText(read, "mode: "+str_velocity_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

        click_flag = False
        if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN:
            click_flag = True
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
            if velocity_mode == 0:
                cv2.drawMarker(read, paths[min_number][min_index] ,(240, 240, 240), markerType=cv2.MARKER_STAR, markerSize=15)
                paths_map["path"+ str(min_number)]["point"+ str(min_index).zfill(3)].update(velocity_straight)
            elif velocity_mode == 1:
                cv2.drawMarker(read, paths[min_number][min_index] ,(100, 100, 240), markerType=cv2.MARKER_STAR, markerSize=15)
                paths_map["path"+ str(min_number)]["point"+ str(min_index).zfill(3)].update(velocity_curve)
            elif velocity_mode == 2:
                cv2.drawMarker(read, paths[min_number][min_index] ,(240, 100, 100), markerType=cv2.MARKER_STAR, markerSize=15)
                paths_map["path"+ str(min_number)]["point"+ str(min_index).zfill(3)].update(velocity_junction)

        while click_flag:
            time.sleep(0.01)
            if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN or mouseData.getEvent() == cv2.EVENT_RBUTTONDOWN:
                break

    cv2.destroyAllWindows()

    print("enter paths(.yaml)")
    path_connection_filename = raw_input(">>")
    f = open(save_filepath + path_connection_filename, "w")

    f.write("%YAML 1.2\n")
    f.write("---\n")
    f.write(yaml.dump(velocity, default_flow_style=False))

    f.write(yaml.dump(paths_map, default_flow_style=False))
    f.close()

    with open(save_filepath + path_connection_filename) as fs:
        data_lines = fs.read()
        data_lines = data_lines.replace("./paths_constraint0.png", s + "./paths_constraint0.png" + s)
        data_lines = data_lines.replace("./paths_constraint1.png", s + "./paths_constraint1.png" + s)
        data_lines = data_lines.replace("./paths_constraint2.png", s + "./paths_constraint2.png" + s)
        data_lines = data_lines.replace("./paths_constraint3.png", s + "./paths_constraint3.png" + s)
        data_lines = data_lines.replace("./paths_constraint4.png", s + "./paths_constraint4.png" + s)
        data_lines = data_lines.replace("'", "")

    with open(save_filepath + path_connection_filename, mode="w") as fs:
        fs.write(data_lines)
