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

def yaml_pass_conn_read(path,index):
        return (path["conn"+ str(index).zfill(3)]["src_path_index"],
                path["conn"+ str(index).zfill(3)]["src_path_number"],
                path["conn"+ str(index).zfill(3)]["dst_path_index"],
                path["conn"+ str(index).zfill(3)]["dst_path_number"]
        )

def pass_conn2point(path,index,path_number):
        return path[path_number][index]

def get_norm(point1,point2):
        absx = abs(point1[0] - point2[0])
        absy = abs(point1[1] - point2[1])
        return math.sqrt(absx*absx + absy*absy)

def varidate_conn(paths, path_conn_info, src_path_number, src_path_index, dst_path_number, dst_path_index):
    if src_path_number == dst_path_number:
        if dst_path_index == src_path_index + 1 or (dst_path_index == 0 and src_path_index == len(paths[src_path_number]) - 1):
            return True
    else:
        for i in range(path_conn_info["num"]):
            conn_info = path_conn_info["conn" + str(i).zfill(2)]
            if src_path_number == conn_info["src_path_number"] and src_path_index  == conn_info["src_path_index"] and dst_path_number == conn_info["dst_path_number"] and dst_path_index == conn_info["dst_path_index"]:
                return True
    return False

if __name__ == "__main__":
    read_filepath = "../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/"
    save_filepath = "../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/route/"
    #print("enter img_filepath(.png)")
    # img_filepath = raw_input(">>")
    # read = cv2.imread(read_filepath + img_filepath)
    read = cv2.imread(read_filepath + "line.png")

    window_name = "input window"
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

    # read paths yaml
    print("enter paths_filepath(.yaml)")
    paths_filepath = raw_input(">>")
    fs = open(read_filepath + paths_filepath, "r")
    # fs = open("../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/paths1.yaml", "r")

    # read path_connection yaml
    print("enter path_connection_filepath(.yaml)")
    path_connection_filepath = raw_input(">>")
    fs2 = open(read_filepath + path_connection_filepath, "r")
    # fs2 = open("../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/path_connection4.yaml", "r")

    paths_map = yaml.load(fs)
    path_conn_info = yaml.load(fs2)
    fs.close()
    fs2.close()

    paths = []
    for i in range(paths_map["num"]):
        path = []
        for j in range(paths_map["path" + str(i)]["num"]):
            point = yaml_point_read(paths_map["path" + str(i)], j)
            one_based_index = i + 1
            cv2.circle(read, point, 4, ((one_based_index & 0b001) * 255, (one_based_index & 0b010) * 255, (one_based_index & 0b100) * 255), -1)
            path.append(point)
        paths.append(path)

    for i in range(path_conn_info["num"]):
        conn_info = path_conn_info["conn" + str(i).zfill(2)]
        src = paths[conn_info["src_path_number"]][conn_info["src_path_index"]]
        dst = paths[conn_info["dst_path_number"]][conn_info["dst_path_index"]]
        cv2.drawMarker(read, src,(240, 240, 240), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15)
        cv2.drawMarker(read, dst,(240, 240, 240), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15)
        cv2.arrowedLine(read, src, dst, (0, 255, 255), thickness=2)

    window_name = "input window"
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

    mouseData = mouseParam(window_name)

    route_number = []
    route_index = []
    ref_point = []
    route = []
    cnt = 0

    cv2.putText(read, "quit: q", (300,  50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "save: s", (300,  80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "is_not_via point : leftclick" , (20,  200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "is_via_point     : rightclick" , (20,  250), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    while 1:
        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit()

        cv2.imshow(window_name, read)
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break

        click_flag = False
        if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN or mouseData.getEvent() == cv2.EVENT_RBUTTONDOWN:
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
            if cnt == 0:
                route_number.append(min_number)
                route_index.append(min_index)
                if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN:
                    cv2.drawMarker(read, paths[min_number][min_index] ,(100, 240, 100), markerType=cv2.MARKER_STAR, markerSize=20)
                    data = {"number": min_number, "index": min_index, "is_via_point": 0}
                    route.append(data)
                else:
                    cv2.drawMarker(read, paths[min_number][min_index] ,(100, 100, 200), markerType=cv2.MARKER_STAR, markerSize=20)
                    data = {"number": min_number, "index": min_index, "is_via_point": 1}
                    route.append(data)
                cnt += 1
            elif varidate_conn(paths, path_conn_info, route_number[cnt-1], route_index[cnt-1], min_number, min_index):
                route_number.append(min_number)
                route_index.append(min_index)
                if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN:
                    cv2.drawMarker(read, paths[min_number][min_index] ,(100, 240, 100), markerType=cv2.MARKER_STAR, markerSize=20)
                    cv2.arrowedLine(read, paths[route_number[cnt-1]][route_index[cnt-1]], paths[min_number][min_index], (128, 128, 128), thickness=2)
                    data = {"number": min_number, "index": min_index, "is_via_point": 0}
                    route.append(data)
                else:
                    cv2.drawMarker(read, paths[min_number][min_index] ,(100, 100, 200), markerType=cv2.MARKER_STAR, markerSize=20)
                    cv2.arrowedLine(read, paths[route_number[cnt-1]][route_index[cnt-1]], paths[min_number][min_index], (128, 128, 128), thickness=2)
                    data = {"number": min_number, "index": min_index, "is_via_point": 1}
                    route.append(data)
                cnt += 1

        while click_flag:
            time.sleep(0.01)
            if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN or mouseData.getEvent() == cv2.EVENT_RBUTTONDOWN:
                break


    cv2.destroyAllWindows()

    print("enter route_filepath(.yaml)")
    route_filename = raw_input(">>")
    f = open(save_filepath + route_filename, "w")
    # f = open("../../ultra96/ROOT_FS/app/fad/data/RoutePlanner/route4.yaml", "w")
    f.write("%YAML 1.2\n")
    f.write("---\n")
    for i in range(len(route)):
        write_data = {"ref_point"+str(i).zfill(3): route[i]}
        f.write(yaml.dump(write_data, default_flow_style=False))
    f.write("num: " + str(cnt))
    f.close()

    print("Finished")
