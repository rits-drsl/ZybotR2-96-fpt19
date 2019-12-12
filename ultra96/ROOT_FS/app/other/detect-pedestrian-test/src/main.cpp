/**
 *  detect-pedestrian-test:
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "main.h"

bool detect_pd(cv::Mat src_img, cv::Mat dst_img) {
    // cv::Mat dst_img = src_img.clone();

    std::string cascade_file = "../data/pd.xml";

    cv::CascadeClassifier cascade;

    cascade.load(cascade_file);

    if(cascade.empty()) {
        std::cerr << "cannot load cascade file" << std::endl;
        exit(-1);
    }

    std::vector<cv::Rect> objects;
    cascade.detectMultiScale(src_img, objects, 1.1, 5);

    std::vector<cv::Mat> rect_imgs;
    for(auto& o : objects) {
        // std::cout << "(x, y, width, height) = (" << o.x << ", " << o.y << ", "
        //     << o.width << ", " << o.height << ")" << std::endl;
        // cv::rectangle(dst_img,cv::Rect(o.x, o.y, o.width, o.height), cv::Scalar(0, 0, 255), 2);
        cv::Mat rect_img(src_img, cv::Rect(o.x, o.y, o.width, o.height));
        rect_imgs.push_back(rect_img);
    }
    // cv::imwrite("result.png", dst_img);


    // バイラテラルフィルタ(処理時間厳しいので諦め)
    // vector<cv::Mat> dst_rect_imgs;
    // for (size_t i = 0; i < rect_imgs.size(); i++) {
    //     cv::Mat out;
    //     auto s = chrono::system_clock::now();
    //     cv::bilateralFilter(rect_imgs[i], out, 5, 100, 10);
    //     auto e = chrono::system_clock::now();
    //     std::cout << "elapsed time(ms) : "
    //               << std::chrono::duration_cast<std::chrono::microseconds>(e - s).count() / 1000.0
    //               << std::endl;

    //     cv::imwrite("bilateral" + std::to_string(i) + ".png", out);
    //     dst_rect_imgs.push_back(out);
    // }

    bool is_pd = false;
    int area_threshold = 5000;
    double aspect_ratio_threshold = 2.5;
    // HSV
    auto MIN_HSVCOLOR = cv::Scalar(0, 30, 60);
    auto MAX_HSVCOLOR = cv::Scalar(20, 150, 255);
    cv::Mat kernel(5,5,CV_8U);
    for (size_t i = 0; i < rect_imgs.size(); i++) {
        cv::GaussianBlur(rect_imgs[i], rect_imgs[i], cv::Size(5, 5), 0);
        cv::cvtColor(rect_imgs[i], rect_imgs[i], cv::COLOR_BGR2HSV);
        cv::inRange(rect_imgs[i], MIN_HSVCOLOR, MAX_HSVCOLOR, rect_imgs[i]);
        cv::morphologyEx(rect_imgs[i], rect_imgs[i], cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 3);
        cv::Mat labeled_ract_img;
        std::map<uint16_t, improc::LabelingExecutor::Info> label_info_map;
        improc::LabelingExecutor::execute(rect_imgs[i], labeled_ract_img, label_info_map);
        auto begin = label_info_map.begin(), end = label_info_map.end();
            for (auto iter = begin; iter != end; iter++) {
                double width  = std::abs(iter->second.end.x - iter->second.begin.x);
                double height = std::abs(iter->second.end.y - iter->second.begin.y);
                auto aspect_ratio = height / width;
                // std::cout << "area= " << iter->second.area << std::endl;
                // std::cout << "width : " << width << std::endl;
                // std::cout << "height : " << height << std::endl;
                // std::cout << "aspect ratio= " << height / width << std::endl;
                if(iter->second.area > area_threshold && aspect_ratio > aspect_ratio_threshold) {
                    is_pd = true;
                    dst_img = rect_imgs[i];
                }
        }
    }

    if(is_pd) {
        return true;
    } else {
        return false;
    }

    // 出力
    for (size_t i = 0; i < rect_imgs.size(); i++) {
        cv::imwrite("result" + std::to_string(i) + ".png", rect_imgs[i]);
    }
}

int main() {
    // cv::Mat src_img = cv::imread("../data/pd.png");
    // if(src_img.empty()) {
    //     std::cerr << "cannot load image" << std::endl;
    //     exit(-1);
    // }
    // cv::Mat dst_img;
    // detect_pd(dst_img, dst_img);

    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        exit(-1);
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
        cv::Mat result;

        auto judgment = detect_pd(corrected_frame,result);
        if (judgment){
            std::cout << "detect" << std::endl;
        }else{
            std::cout << "not detect" << std::endl;
        }
        cv::imshow("result", result);


        const int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
        else if(key == 's') {
            cv::imwrite("img" + std::to_string(index) + ".png", corrected_frame);
            index++;
        }
    }
    cv::destroyAllWindows();

    return 0;
}