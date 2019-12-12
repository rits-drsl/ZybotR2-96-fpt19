/**
 *  AKAZEPatternMatcher: A-KAZE特徴量抽出によるパターンマッチングを行い、
 *                       入力座標とのずれを求める
 *
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef IMPROC_INCLUDE_AKAZEPATTERNMATCHER_AKAZEPATTERNMATCHER_H_
#define IMPROC_INCLUDE_AKAZEPATTERNMATCHER_AKAZEPATTERNMATCHER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <chrono>

namespace improc {
    class AKAZEPatternMatcher {
    public:
        struct Delta {
            double x;
            double y;
            double theta;
        };

        struct FeatureInfo {
            std::vector<cv::KeyPoint> kpts;
            cv::Mat                   desc;
        };

        AKAZEPatternMatcher(int         descriptor_type       = cv::AKAZE::DESCRIPTOR_MLDB_UPRIGHT,
                            int         descriptor_size       = 0,
                            int         descriptor_channels   = 3,
                            float       threshold             = 0.001f,
                            int         nOctaves              = 4,
                            int         nOctaveLayers         = 4,
                            int         diffusivity           = cv::KAZE::DIFF_PM_G2,
                            std::string DescriptorMatcherType = "BruteForce-Hamming",
                            double      NNDR                  = 0.8,
                            float       matching_dist_thr     = 0.80f);

        ~AKAZEPatternMatcher();

        AKAZEPatternMatcher(const AKAZEPatternMatcher& obj) = delete;
        AKAZEPatternMatcher &operator=(const AKAZEPatternMatcher& obj) = delete;

        /**
         *  A-KAZE特徴量抽出を実行する
         *  @src    : 入力画像
         *  @result : 結果
         *  @Return : 特徴点が検出できなかった場合、falseを返す
         */
        bool computeFeatureInfo(const cv::Mat& src,
                                FeatureInfo& result) const;

        /**
         *  A-KAZE特徴量抽出によるパターンマッチングを行い、画像間の変位を導出する
         *  @ref           : 参照画像
         *  @point         : 参照画像において、対象画像の中心座標が相当する座標
         *  @target        : 対象画像(pointを中心に取る)
         *  @min_dist_thr  : 誤差のしきい値
         *  @imshow_result : 結果をcv::imshowを使用して表示するかどうかのフラグ
         *  @wait          : cv::waitKeyの引数
         *  @Return        : マッチングが成功した場合はtrueを返す
         *                   (具体的には変位のノルムがmin_dist_thr以下だった場合)
         */
        bool execute(const cv::Mat&     ref,
                     const cv::Point2f& point,
                     const cv::Mat&     target,
                     const double&      min_dist_thr,
                     const bool&        imshow_result = false,
                     const uint32_t&    wait          = 0);

        /**
         *  A-KAZE特徴量抽出によるパターンマッチングを行い、画像間の変位を導出する
         *  @ref_feature     : 参照画像の特徴量の情報
         *  @ref_img_size    : 参照画像のサイズ
         *  @point           : 上記参照
         *  @target_feature  : 対象画像の特徴量の情報
         *  @target_img_size : 対象画像のサイズ
         *  @min_dist_thr    : 上記参照
         *  @imshow_result   : 上記参照
         *  @wait            : 上記参照
         *  @ref_img         : 参照画像(結果をcv::imshowで表示する場合は設定すること)
         *  @target_img      : 対象画像(結果をcv::imshowで表示する場合は設定すること)
         */
        bool execute(const FeatureInfo& ref_feature,
                     const cv::Size&    ref_img_size,
                     const cv::Point2f& point,
                     const FeatureInfo& target_feature,
                     const cv::Size&    target_img_size,
                     const double&      min_dist_thr,
                     const bool&        imshow_result   = false,
                     const uint32_t&    wait            = 0,
                     const cv::Mat&     ref_img         = cv::Mat(),
                     const cv::Mat&     target_img      = cv::Mat());

        const Delta& getResult() const;

    private:
        cv::Ptr<cv::AKAZE> akaze_;
        Delta              result_;
        std::string        DescriptorMatcherType_;
        double             NNDR_;
        float              matching_dist_thr_;

        /**
         *  対象の点に対して、任意点周りの回転＋平行移動を行う
         *  @pt        : 対象の点
         *  @center_pt : 任意点
         *  @dpt       : x方向の移動量, y方向の移動量
         *  @dtheta    : 回転角
         *  @Return    : 移動後の点
         */
        cv::Point2f trans(const cv::Point2f& pt,
                          const cv::Point2f& center_pt,
                          const cv::Point2f& dpt,
                          const double&      dtheta) const;

        /**
         *  対象の点に対して、任意点周りの回転＋平行移動を行う
         *  @pt     : 対象の点
         *  @trans1 : 上記trans関数の実装を参照
         *  @trans2 : 上記trans関数の実装を参照
         *  @dtheta : 回転角
         *  @Return : 移動後の点
         */
        cv::Point2f trans(const cv::Point2f&                  pt,
                          const Eigen::Translation<float, 2>& trans1,
                          const Eigen::Translation<float, 2>& trans2,
                          const double&                       dtheta) const;
    };
}

#endif /* IMPROC_INCLUDE_AKAZEPATTERNMATCHER_AKAZEPATTERNMATCHER_H_ */
