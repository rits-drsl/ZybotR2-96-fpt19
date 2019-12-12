/**
 *  optor-test:
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "main.h"

using namespace std;
using namespace cv;

bool close_img_viewer = false;

timeval left_stamp, right_stamp;

void optor_test() {
    visensor_load_settings("../data/param.txt");

    if(visensor_Start_Cameras() < 0) {
        printf("Opening cameras failed...\r\n");
        return;
    }

    if(visensor_Start_IMU() < 0) {
        printf("visensor_open_port error...\r\n");
        return;
    }

    usleep(100000);

    Mat img_left(Size(visensor_img_width(), visensor_img_height()), CV_8UC1);
    Mat img_right(Size(visensor_img_width(), visensor_img_height()), CV_8UC1);

    double left_timestamp, right_timestamp;
    while(true) {
        if(visensor_is_leftcam_open()) {
            if(visensor_is_left_img_new()) {
                visensor_get_left_latest_img(img_left.data, &left_timestamp);
                imshow("left",img_left);
            }
        }

        if(visensor_is_rightcam_open()) {
            if(visensor_is_right_img_new()) {
                visensor_get_right_latest_img(img_right.data, &right_timestamp);
                imshow("right",img_right);
            }
        }

        if(visensor_imu_have_fresh_data()) {
            visensor_imudata imudata;
            visensor_get_imudata_latest(&imudata);
            printf("IMUTime:%8.6f, Gyr: %8.4f,%8.4f,%8.4f, Acc: %8.4f,%8.4f,%8.4f, Quat(WXYZ): %8.4f,%8.4f,%8.4f,%8.4f\n",
                   imudata.timestamp,
                   imudata.rx,imudata.ry,imudata.rz,
                   imudata.ax,imudata.ay,imudata.az,
                   imudata.qw,imudata.qx,imudata.qy,imudata.qz);
        }
        usleep(100);

        auto key = waitKey(1);
        if(key == 'q') break;
    }

    visensor_Close_Cameras();
    visensor_Close_IMU();
}

int main() {
    optor_test();
    return 0;
}
