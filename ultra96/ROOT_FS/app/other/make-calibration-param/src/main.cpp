/**
 *  make-calibration-param:
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

int main (int argc, char *argv[]) {
    int num_img;
    cout << "Please input the number of image : ";
    cin >> num_img;

    int board_col;
    cout << "Please input the number of checker board column : ";
    cin >> board_col;

    int board_row;
    cout << "Please input the number of checker board row : ";
    cin >> board_row;

    float square_size;
    cout << "Please input squares size (mm) : ";
    cin >> square_size;

    const Size BOARD_SIZE = Size(board_col, board_row);
    const int  N_CORNERS  = board_col * board_row;

    vector<Mat> src_image(num_img);
    for(int i = 0;i < num_img;i++) {
        src_image[i] = imread("img" + to_string(i) + ".png");
    }

    vector<vector<Point2f>> imagePoints;
    vector<Point2f>         imageCorners;
    Mat                     gray_image;
    Mat                     dst_image;

    namedWindow("corner detect image");
    for(int i = 0; i < num_img; i++) {
        auto found = findChessboardCorners(src_image[i], BOARD_SIZE, imageCorners);
        cvtColor(src_image[i], gray_image, CV_BGR2GRAY);
        cornerSubPix(gray_image, imageCorners, Size(9, 9), Size(1, -1), TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.1));
        if(found) {
            imagePoints.push_back(imageCorners);
        }

        dst_image = src_image[i].clone();
        drawChessboardCorners(dst_image, BOARD_SIZE, imageCorners, found);
        imshow("corner detect image", dst_image);
        waitKey(0);
    }

    vector<vector<Point3f>> objectPoints;
    vector<Point3f>         objectCorners;
    for(int j = 0; j < board_row; j++) {
        for(int i = 0; i < board_col; i++) {
            objectCorners.push_back(Point3f(i*square_size, j*square_size, 0.0f));
        }
    }
    for(int i = 0; i < num_img; i++) {
        objectPoints.push_back(objectCorners);
    }

    Mat         cameraMatrix;
    Mat         distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    auto rms = calibrateCamera(objectPoints,
                               imagePoints,
                               src_image[0].size(),
                               cameraMatrix,
                               distCoeffs,
                               rvecs,
                               tvecs);

    cout << fixed << right;
    cout << "Re-projection Error(uint;pixel)" << endl;
    cout << " " << rms << endl;
    cout << "cameraMatrix(uint;pixel)" << endl;
    cout << " fx=" << cameraMatrix.at<double>(0, 0);
    cout << " fy=" << cameraMatrix.at<double>(1, 1);
    cout << " cx=" << cameraMatrix.at<double>(0, 2);
    cout << " cx=" << cameraMatrix.at<double>(1, 2);
    cout << endl << endl;
    cout << "distCoeffs" << endl;
    cout << " k1=" << distCoeffs.at<double>(0, 0);
    cout << " k2=" << distCoeffs.at<double>(0, 1);
    cout << " p1=" << distCoeffs.at<double>(0, 2);
    cout << " p1=" << distCoeffs.at<double>(0, 3);
    cout << " k3=" << distCoeffs.at<double>(0, 4);
    cout << endl << endl;

    FileStorage fs("calib_param.xml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    undistort(src_image[0], dst_image, cameraMatrix, distCoeffs);
    namedWindow("calibration image");
    imshow("calibration image", dst_image);
    waitKey(0);
    destroyAllWindows();

    return 0;
}
