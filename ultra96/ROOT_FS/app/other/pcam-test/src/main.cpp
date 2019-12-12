/**
 *  PCam-test:
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

enum class PreimprocMode {
    GRAY,
    BIN,
    EDGE,
    SAT
};

void pcam_test() {

    double H[9] = {0.44493910474142623, -0.265170158024246, -23.667274634287185,
                   -0.010483730367221628, 0.03984000976025845, 44.47837644610684,
                   -3.556149627457038e-05, -0.0004076085446486301, 0.4412436729698192};


    zynqpl::PreImProc preimproc("uio3");
    preimproc.SetGrayBinThreshold(224);
    preimproc.SetHistThreshold(100, 80);
    preimproc.SetSatBinThreshold(50);

    zynqpl::Pcam pcam("pcam0",
                      "i2c-4",
                      zynqpl::OV5640_cfg::mode_t::MODE_720P_1280_720_60fps,
                      zynqpl::OV5640_cfg::awb_t::AWB_ADVANCED,
                      V4L2_PIX_FMT_GREY);

    cv::Mat img(cv::Size(pcam.getImageWidth(), pcam.getImageHeight()), CV_8UC1);
    auto img_corrected = img;

    improc::ImageCorrector corrector(img.size(), "../data/pcam_calibration_parameter.xml", H);


    cv::namedWindow("frame buffer", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    std::cout << "s : Stop" << std::endl;
    std::cout << "p : Prot center point" << std::endl;
    std::cout << "c : Change mode" << std::endl;
    std::cout << "w : write image" << std::endl;
    std::cout << "q : Quit" << std::endl;

    PreimprocMode mode            = PreimprocMode::GRAY;
    int           write_img_index = 0;
    bool          end_flag        = false;
    bool          prot_center     = false;
    while(!end_flag) {
        pcam.fetchFrame(img.data);
        img_corrected = img.clone();
        corrector.execute(img, img_corrected);

        // 表示モードの切り替え
        switch(mode) {
            case PreimprocMode::GRAY: {
                break;
            }
            case PreimprocMode::BIN: {
                for(int i = 0; i < img_corrected.size().area(); i++) {
                    img_corrected.data[i] = (img_corrected.data[i] & 0b00000001) ? 0xFF : 0x00;
                }
                break;
            }
            case PreimprocMode::EDGE: {
                for(int i = 0; i < img_corrected.size().area(); i++) {
                    img_corrected.data[i] = (img_corrected.data[i] & 0b00000010) ? 0xFF : 0x00;
                }
                break;
            }
            case PreimprocMode::SAT: {
                for(int i = 0; i < img_corrected.size().area(); i++) {
                    img_corrected.data[i] = (img_corrected.data[i] & 0b00000100) ? 0xFF : 0x00;
                }
                break;
            }
        }

        if(prot_center) {
            cv::circle(img_corrected, cv::Point(img_corrected.cols/2, img_corrected.rows/2), 10, cv::Scalar(255, 255, 255));
        }

        cv::imshow("frame buffer", img_corrected);
        auto key = cv::waitKey(100);
        switch(key) {
            case 's': {
                // s : Stop
                while(true) {
                    if(cv::waitKey(0)) {
                        break;
                    }
                }
                break;
            }
            case 'p': {
                // p : Prot center point
                prot_center = !prot_center;
                break;
            }
            case 'c': {
                // c : Change
                mode = static_cast<PreimprocMode>((static_cast<int>(mode) + 1) % 4);
                break;
            }
            case 'w': {
                // w : Write image
                cv::imwrite("image" + std::to_string(write_img_index) + ".png", img_corrected);
                write_img_index++;
                break;
            }
            case 'q': {
                // q : Quit
                end_flag = true;
                break;
            }
            default: {
                break;
            }
        }
    }
    cv::destroyAllWindows();
}

int main() {
    pcam_test();
    return 0;
}
