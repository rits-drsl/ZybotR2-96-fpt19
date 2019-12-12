#include "main.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const string REF_IMG      = "../data/test_real_ref6.png";
const string TARGET_IMG   = "../data/test_real_tar6.png";
const Point  POINT        = Point(0, 0);
const int    MIN_DIST_THR = 10000;
const int    LOOP_CNT     = 1;
const bool   SHOW_RESULT  = true;

Point2f trans(const Point& pt, const Point2f& center_pt, const int& dx, const int& dy, const double& dtheta) {
    auto rotate       = Rotation2Df(dtheta);
    auto translation1 = Translation<float, 2>(center_pt.x + dx, center_pt.y + dy);
    auto translation2 = Translation<float, 2>(-center_pt.x, -center_pt.y);
    auto mat = translation2 * rotate * translation1;
    Vector2f projection = mat * Vector2f(pt.x, pt.y);
    return Point2f(projection.x(), projection.y());
}

int main() {
    Mat ref    = imread(REF_IMG);
    Mat target = imread(TARGET_IMG);
    Mat out    = ref.clone();

    improc::AKAZEPatternMatcher matcher(AKAZE::DESCRIPTOR_MLDB_UPRIGHT,
                                        0,
                                        3,
                                        0.00001f,
                                        4,
                                        4,
                                        KAZE::DIFF_PM_G2,
                                        "BruteForce-Hamming",
                                        0.8,
                                        0.80f);

    double time_sum_ms = 0;
    for(int i = 0; i < LOOP_CNT; i++) {
        auto s       = std::chrono::system_clock::now();
        auto status  = matcher.execute(ref, POINT, target, MIN_DIST_THR, SHOW_RESULT);
        auto e       = std::chrono::system_clock::now();
        auto time_ms = std::chrono::duration_cast<std::chrono::microseconds>(e - s).count() / 1000;

        if(status && SHOW_RESULT) {
            auto result = matcher.getResult();
            std::cout << "ans.dx : " << result.x << std::endl;
            std::cout << "ans.dy : " << result.y << std::endl;
            std::cout << "ans.dtheta : " << result.theta << std::endl;

            Point estimate_point(POINT.x + result.x, POINT.y + result.y);
            circle(out, estimate_point, 3, Scalar(0, 255, 255), -1, CV_AA);
            Mat R = (Mat_<float>(2,2) <<
                     cos(result.theta), -sin(result.theta),
                     sin(result.theta),  cos(result.theta));

            Point pt1 = POINT + Point(-target.cols / 2, -target.rows / 2);
            Point pt2 = POINT + Point( target.cols / 2, -target.rows / 2);
            Point pt3 = POINT + Point(-target.cols / 2,  target.rows / 2);
            Point pt4 = POINT + Point( target.cols / 2,  target.rows / 2);

            Point rpt1 = trans(pt1, POINT, result.x, result.y, result.theta);
            Point rpt2 = trans(pt2, POINT, result.x, result.y, result.theta);
            Point rpt3 = trans(pt3, POINT, result.x, result.y, result.theta);
            Point rpt4 = trans(pt4, POINT, result.x, result.y, result.theta);

            circle(out, rpt1, 5, Scalar(0,   255,   0), 1, CV_AA);
            circle(out, rpt2, 5, Scalar(255,   0,   0), 1, CV_AA);
            circle(out, rpt3, 5, Scalar(0,     0, 255), 1, CV_AA);
            circle(out, rpt4, 5, Scalar(0,     0,   0), 1, CV_AA);

            line(out, rpt1, rpt2, Scalar(255, 255, 0), 1, LINE_AA, 0);
            line(out, rpt2, rpt4, Scalar(255, 255, 0), 1, LINE_AA, 0);
            line(out, rpt4, rpt3, Scalar(255, 255, 0), 1, LINE_AA, 0);
            line(out, rpt3, rpt1, Scalar(255, 255, 0), 1, LINE_AA, 0);

            namedWindow("tar", WINDOW_KEEPRATIO | WINDOW_NORMAL);
            namedWindow("out", WINDOW_KEEPRATIO | WINDOW_NORMAL);
            imshow("tar", target);
            imshow("out", out);
            waitKey(0);
            destroyAllWindows();
        }
        else if(!status) {
            std::cout << " did not match " << std::endl;
        }

        time_sum_ms += time_ms;
        std::cout << "execute() elapsed time(ms) : " << time_ms << std::endl;
    }
    std::cout << "time(ms) : " << time_sum_ms / LOOP_CNT << std::endl;

    return 0;
}
