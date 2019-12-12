/**
 *  webcam-test:
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "main.h"

void webcam_test() {
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        return;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    improc::ImageCorrector corr(cv::Size(640, 480), "../data/VGA/calib_param.xml");

    cv::Mat frame, corrected_frame;

    cap.read(frame);
    corrected_frame = frame.clone();

    int index = 0;
    while(cap.read(frame)) {
        corr.execute(frame, corrected_frame);
        cv::imshow("corrected_frame", corrected_frame);

        const int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
        else if(key == 's') {
            std::cout << "write image as \"img" + std::to_string(index) + ".png\"" << std::endl;
            cv::imwrite("img" + std::to_string(index) + ".png", corrected_frame);
            index++;
        }
    }
    cv::destroyAllWindows();
}

int main() {
    webcam_test();
    return 0;
}
