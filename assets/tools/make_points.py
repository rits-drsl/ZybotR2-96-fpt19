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
# point on the local map ---> write yaml

# To Use
# $ sudo apt install python-pip
# $ pip install python-opencv
# $ pip install yaml

import math
import time
import cv2
import yaml

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
    rows , cols, c = read.shape

    candidate_points = []

    mark_color = (128, 128, 128)

    for i in range(0, cols, 10):
        for j in range(0, rows, 10):
            cv2.circle(read, (i, j), 3, color=mark_color, thickness=-1)
            candidate_points.append((i,j))


    window_name = "input window"
    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, read)

    mouseData = mouseParam(window_name)

    yaml_data0 = []
    yaml_data1 = []
    yaml_data2 = []
    yaml_data3 = []
    yaml_data4 = []

    point_num0 = 0
    point_num1 = 0
    point_num2 = 0
    point_num3 = 0
    point_num4 = 0

    mode = 0
    str_mode = str(mode)

    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "mode0: outside"     , (800,  50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "mode1: upper   left", (800,  80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "mode2: upper  right", (800, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "mode3: bottom  left", (800, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "mode4: bottom right", (800, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "quit: q", (300,  50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "next: n", (300,  80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "back: b", (300, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    cv2.putText(read, "is not stop line : leftclick" , (20,  200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
    cv2.putText(read, "is stop line     : rightclick", (20,  250), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)

    while mode < 5:
        while 1:
            cv2.imshow(window_name, read)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                exit()

            cv2.imshow(window_name, read)
            if cv2.waitKey(1) & 0xFF == ord("n"):
                cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                mode = mode + 1
                str_mode = str(mode)
                cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                break

            cv2.imshow(window_name, read)
            if cv2.waitKey(1) & 0xFF == ord("b"):
                if mode == 0:
                    point_num0 = 0
                    for v in yaml_data0:
                        for w in v.values():
                            delete_pos = w.items()
                            cv2.circle(read, (delete_pos[1][1], delete_pos[0][1]), 3, mark_color, -1)
                    yaml_data0 = []
                    break
                if mode == 1:
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                    point_num1 = 0
                    for v in yaml_data1:
                        for w in v.values():
                            delete_pos = w.items()
                            cv2.circle(read, (delete_pos[1][1], delete_pos[0][1]), 3, mark_color, -1)
                    yaml_data1 = []
                    mode = mode - 1
                    str_mode = str(mode)
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                    break
                if mode == 2:
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                    point_num2 = 0
                    for v in yaml_data2:
                        for w in v.values():
                            delete_pos = w.items()
                            cv2.circle(read, (delete_pos[1][1], delete_pos[0][1]), 3, mark_color, -1)
                    yaml_data2 = []
                    mode = mode - 1
                    str_mode = str(mode)
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                    break
                if mode == 3:
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                    point_num3 = 0
                    for v in yaml_data3:
                        for w in v.values():
                            delete_pos = w.items()
                            cv2.circle(read, (delete_pos[1][1], delete_pos[0][1]), 3, mark_color, -1)
                    yaml_data3 = []
                    mode = mode - 1
                    str_mode = str(mode)
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                    break
                if mode == 4:
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), thickness=2)
                    point_num4 = 0
                    for v in yaml_data4:
                        for w in v.values():
                            delete_pos = w.items()
                            cv2.circle(read, (delete_pos[1][1], delete_pos[0][1]), 3, mark_color, -1)
                    yaml_data4 = []
                    mode = mode - 1
                    str_mode = str(mode)
                    cv2.putText(read, "mode: "+str_mode, (500, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness=2)
                    break

            # leftclick: is not stop line
            if mouseData.getEvent() == cv2.EVENT_LBUTTONDOWN:
                time.sleep(0.2)
                min_norm1 = 1000000000
                min_index1 = -1
                lpos = mouseData.getPos()
                for ni in range(len(candidate_points)):
                    norm1 = get_norm(lpos, candidate_points[ni])
                    if norm1 < min_norm1:
                            min_norm1   = norm1
                            min_index1  = ni
                cv2.circle(read, candidate_points[min_index1], 3, (0, 255, 0), -1)
                if mode == 0:
                    data = {"point"+str(point_num0).zfill(3): {"x": candidate_points[min_index1][0], "y": candidate_points[min_index1][1], "is_stop_line": 0, "line_trace_mode": 1}}
                    yaml_data0.append(data)
                    print(candidate_points[min_index1])
                    point_num0 += 1
                if mode == 1:
                    data = {"point"+str(point_num1).zfill(3): {"x": candidate_points[min_index1][0], "y": candidate_points[min_index1][1], "is_stop_line": 0, "line_trace_mode": 0}}
                    yaml_data1.append(data)
                    print(candidate_points[min_index1])
                    point_num1 += 1
                if mode == 2:
                    data = {"point"+str(point_num2).zfill(3): {"x": candidate_points[min_index1][0], "y": candidate_points[min_index1][1], "is_stop_line": 0, "line_trace_mode": 0}}
                    yaml_data2.append(data)
                    print(candidate_points[min_index1])
                    point_num2 += 1
                if mode == 3:
                    data = {"point"+str(point_num3).zfill(3): {"x": candidate_points[min_index1][0], "y": candidate_points[min_index1][1], "is_stop_line": 0, "line_trace_mode": 0}}
                    yaml_data3.append(data)
                    print(candidate_points[min_index1])
                    point_num3 += 1
                if mode == 4 :
                    data = {"point"+str(point_num4).zfill(3): {"x": candidate_points[min_index1][0], "y": candidate_points[min_index1][1], "is_stop_line": 0, "line_trace_mode": 0}}
                    yaml_data4.append(data)
                    print(candidate_points[min_index1])
                    point_num4 += 1
            # centerclick: this is stop line
            elif mouseData.getEvent() == cv2.EVENT_RBUTTONDOWN:
                time.sleep(0.2)
                min_norm2 = 1000000000
                min_index2 = -1
                rpos =  mouseData.getPos()
                for ii in range(len(candidate_points)):
                    norm2 = get_norm(rpos, candidate_points[ii])
                    if norm2 < min_norm2:
                            min_norm2   = norm2
                            min_index2  = ii
                cv2.circle(read, candidate_points[min_index2], 3, (0, 255, 255), -1)
                if mode == 0:
                    data = {"point"+str(point_num0).zfill(3): {"x": candidate_points[min_index2][0], "y": candidate_points[min_index2][1], "is_stop_line": 1, "line_trace_mode": 1}}
                    yaml_data0.append(data)
                    print(candidate_points[min_index2])
                    # for v in yaml_data0:
                    #     for w in v.values():
                    #         a = w.items()
                    #         print(a[0][1])
                    #         print(a[1][1])
                    point_num0 += 1
                if mode == 1:
                    data = {"point"+str(point_num1).zfill(3): {"x": candidate_points[min_index2][0], "y": candidate_points[min_index2][1], "is_stop_line": 1, "line_trace_mode": 0}}
                    yaml_data1.append(data)
                    print(candidate_points[min_index2])
                    point_num1 += 1
                if mode == 2:
                    data = {"point"+str(point_num2).zfill(3): {"x": candidate_points[min_index2][0], "y": candidate_points[min_index2][1], "is_stop_line": 1, "line_trace_mode": 0}}
                    yaml_data2.append(data)
                    print(candidate_points[min_index2])
                    point_num2 += 1
                if mode == 3:
                    data = {"point"+str(point_num3).zfill(3): {"x": candidate_points[min_index2][0], "y": candidate_points[min_index2][1], "is_stop_line": 1, "line_trace_mode": 0}}
                    yaml_data3.append(data)
                    print(candidate_points[min_index2])
                    point_num3 += 1
                if mode == 4 :
                    data = {"point"+str(point_num4).zfill(3): {"x": candidate_points[min_index2][0], "y": candidate_points[min_index2][1], "is_stop_line": 1, "line_trace_mode": 0}}
                    yaml_data4.append(data)
                    print(candidate_points[min_index2])
                    point_num4 += 1

    cv2.destroyAllWindows()

    s = '"'
    print(s)

    # todo
    print("enter filepath(.yaml)")
    filename = raw_input(">>")

    f = open(save_filepath + filename, "w")

    f.write("%YAML 1.2\n")
    f.write("---\n")


    path0 = {}
    path0["path0"] = []
    path0["path0"].append({"constraint_file" : s + "/data/RoutePlanner/paths_constraint0.png" + s})
    for i in range(len(yaml_data0)):
        path0["path0"].append(yaml_data0[i])
    path0["path0"].append({"num" : point_num0})
    f.write(yaml.dump(path0, default_flow_style=False))

    path1 = {}
    path1["path1"] = []
    path1["path1"].append({"constraint_file" : s + "/data/RoutePlanner/paths_constraint1.png" + s})
    for i in range(len(yaml_data1)):
        path1["path1"].append(yaml_data1[i])
    path1["path1"].append({"num" : point_num1})
    f.write(yaml.dump(path1, default_flow_style=False))

    path2 = {}
    path2["path2"] = []
    path2["path2"].append({"constraint_file" : s + "/data/RoutePlanner/paths_constraint2.png" + s})
    for i in range(len(yaml_data2)):
        path2["path2"].append(yaml_data2[i])
    path2["path2"].append({"num" : point_num2})
    f.write(yaml.dump(path2, default_flow_style=False))

    path3 = {}
    path3["path3"] = []
    path3["path3"].append({"constraint_file" : s + "/data/RoutePlanner/paths_constraint3.png" + s})
    for i in range(len(yaml_data3)):
        path3["path3"].append(yaml_data3[i])
    path3["path3"].append({"num" : point_num3})
    f.write(yaml.dump(path3, default_flow_style=False))

    path4 = {}
    path4["path4"] = []
    path4["path4"].append({"constraint_file" : s + "/data/RoutePlanner/paths_constraint4.png" + s})
    for i in range(len(yaml_data4)):
        path4["path4"].append(yaml_data4[i])
    path4["path4"].append({"num" : point_num4})
    f.write(yaml.dump(path4, default_flow_style=False))

    f.write("num: 5")
    f.close()

    with open(save_filepath + filename) as fs:
        data_lines = fs.read()
        data_lines = data_lines.replace("-", " ")
        data_lines = data_lines.replace(" ", "-", 4)
        data_lines = data_lines.replace("-", " ", 1)
        data_lines = data_lines.replace("'", "")

    with open(save_filepath + filename, mode="w") as fs:
        fs.write(data_lines)

    print("Finished")

