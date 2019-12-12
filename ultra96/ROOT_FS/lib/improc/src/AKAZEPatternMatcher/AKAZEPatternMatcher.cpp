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

#include <AKAZEPatternMatcher/AKAZEPatternMatcher.h>

namespace improc {
    AKAZEPatternMatcher::AKAZEPatternMatcher(int         descriptor_type,
                                             int         descriptor_size,
                                             int         descriptor_channels,
                                             float       threshold,
                                             int         nOctaves,
                                             int         nOctaveLayers,
                                             int         diffusivity,
                                             std::string DescriptorMatcherType,
                                             double      NNDR,
                                             float       matching_dist_thr) :
        akaze_(cv::AKAZE::create(descriptor_type,
                                 descriptor_size,
                                 descriptor_channels,
                                 threshold,
                                 nOctaves,
                                 nOctaveLayers,
                                 diffusivity)),
        DescriptorMatcherType_(DescriptorMatcherType),
        NNDR_(NNDR),
        matching_dist_thr_(matching_dist_thr){
    }

    AKAZEPatternMatcher::~AKAZEPatternMatcher() {
    }

    bool AKAZEPatternMatcher::computeFeatureInfo(const cv::Mat& src, FeatureInfo& result) const {
        akaze_->detectAndCompute(src, cv::noArray(), result.kpts, result.desc);
        return result.kpts.size() == 0 ? false : true;
    }

    bool AKAZEPatternMatcher::execute(const cv::Mat&     ref,
                                      const cv::Point2f& point,
                                      const cv::Mat&     target,
                                      const double&      min_dist_thr,
                                      const bool&        imshow_result,
                                      const uint32_t&    wait) {
        // A-KAZE特徴量抽出を実行する
        FeatureInfo ref_feature, target_feature;
        auto ref_detect_status    = computeFeatureInfo(ref, ref_feature);
        auto target_detect_status = computeFeatureInfo(target, target_feature);
        if(!ref_detect_status || !target_detect_status) {
            return false;
        }

        // 特徴量マッチングを行い、変位を導出する
        return execute(ref_feature, ref.size(), point,
                       target_feature, target.size(),
                       min_dist_thr,imshow_result, wait, ref, target);
    }

    bool AKAZEPatternMatcher::execute(const FeatureInfo& ref_feature,
                                      const cv::Size&    ref_img_size,
                                      const cv::Point2f& point,
                                      const FeatureInfo& target_feature,
                                      const cv::Size&    target_img_size,
                                      const double&      min_dist_thr,
                                      const bool&        imshow_result,
                                      const uint32_t&    wait,
                                      const cv::Mat&     ref_img,
                                      const cv::Mat&     target_img) {
        // targetの特徴点の座標をrefの位置(point)に従って正規化
        std::vector<cv::Point2f> normalized_target_kpts_pt;
        normalized_target_kpts_pt.reserve(target_feature.kpts.size());
        auto target_center  = cv::Point2f(target_img_size.width / 2, target_img_size.height / 2);
        auto correction_vec = point - target_center;
        for(size_t i = 0; i < target_feature.kpts.size(); i++) {
            normalized_target_kpts_pt.push_back(target_feature.kpts[i].pt + correction_vec);
        }

        // 各特徴点について2近傍のマッチングを計算する
        std::vector<std::vector<cv::DMatch>> dmatches;
        auto matcher = cv::DescriptorMatcher::create(DescriptorMatcherType_);
        matcher->knnMatch(ref_feature.desc, target_feature.desc, dmatches, 2);

        // 対応点を絞る
        std::vector<cv::DMatch>  good_matches;
        std::vector<cv::Point2f> match_point_ref;
        std::vector<cv::Point2f> match_point_tar;
        for(size_t i = 0; i < dmatches.size(); ++i) {
            auto dist1 = dmatches[i][0].distance;
            auto dist2 = dmatches[i][1].distance;

            // 最近傍の点とその次に相似している点の相似具合が
            // ある程度離れているもののみを取得する
            if(dist1 <= dist2 * matching_dist_thr_) {
                good_matches.push_back(dmatches[i][0]);
                match_point_ref.push_back(ref_feature.kpts[dmatches[i][0].queryIdx].pt);
                match_point_tar.push_back(normalized_target_kpts_pt[dmatches[i][0].trainIdx]);
            }
        }

        if(imshow_result) {
            cv::Mat match_result_img;
            // すべての対応点を表示
            cv::drawMatches(ref_img, ref_feature.kpts, target_img, target_feature.kpts, dmatches, match_result_img);
            cv::namedWindow("A-KAZE Matching", cv::WINDOW_KEEPRATIO);
            cv::imshow("A-KAZE Matching", match_result_img);
            cv::waitKey(wait);
            // 絞った後の対応点を表示
            cv::drawMatches(ref_img, ref_feature.kpts, target_img, target_feature.kpts, good_matches, match_result_img);
            cv::namedWindow("A-KAZE Matching", cv::WINDOW_KEEPRATIO);
            cv::imshow("A-KAZE Matching", match_result_img);
            cv::waitKey(wait);
        }

        // RANSACを用いて射影変換行列を推定
        cv::Mat masks;
        cv::Mat H;
        if(match_point_ref.size() != 0 && match_point_tar.size() != 0) {
            H = cv::findHomography(match_point_ref, match_point_tar, masks, cv::RANSAC, 3.f);
        }

        if(H.empty()) {
            return false;
        }

        // RANSACで使われた対応点のみ抽出する
        std::vector<cv::DMatch> inlierMatches;
        std::vector<cv::Point2f> ransac_match_point_ref;
        std::vector<cv::Point2f> ransac_match_point_tar;
        for(auto i = 0; i < masks.rows; ++i) {
            uchar* inlier = masks.ptr<uchar>(i);
            if(inlier[0] == 1) {
                inlierMatches.push_back(good_matches[i]);
            }
        }

        if(imshow_result) {
            // インライアの対応点のみを表示
            cv::Mat match_result_img;
            cv::drawMatches(ref_img, ref_feature.kpts, target_img, target_feature.kpts, inlierMatches, match_result_img);
            cv::namedWindow("A-KAZE Matching", cv::WINDOW_KEEPRATIO);
            cv::imshow("A-KAZE Matching", match_result_img);
            cv::waitKey(wait);
        }

        // インライアの対応点を使用してdx, dy ,dthetaを推定する
        std::vector<std::pair<double, size_t>> inlier_match_points_norm_map;
        for(size_t index = 0; index < inlierMatches.size(); index++) {
            auto& qid = inlierMatches[index].queryIdx;
            auto& tid = inlierMatches[index].trainIdx;
            auto  dpt = ref_feature.kpts[qid].pt - normalized_target_kpts_pt[tid];
            inlier_match_points_norm_map.emplace_back(cv::norm(dpt), index);
        }

        std::sort(inlier_match_points_norm_map.begin(), inlier_match_points_norm_map.end());

        auto& inlier_median_index = inlier_match_points_norm_map[inlier_match_points_norm_map.size() / 2].second;
        auto& inlier_median_qid   = inlierMatches[inlier_median_index].queryIdx;
        auto& inlier_median_tid   = inlierMatches[inlier_median_index].trainIdx;
        auto  inlier_result_dpt   = ref_feature.kpts[inlier_median_qid].pt - normalized_target_kpts_pt[inlier_median_tid];

        double inlier_result_dtheta = 0;
        auto   inlier_min_dist_sum  = std::numeric_limits<double>::max();
        auto   translation1         = Eigen::Translation<float, 2>(point.x + inlier_result_dpt.x, point.y + inlier_result_dpt.y);
        auto   translation2         = Eigen::Translation<float, 2>(-point.x, -point.y);
        for(double dtheta = -3.14; dtheta <= 3.14; dtheta += 0.00314) {
            double inlier_dist_sum = 0;
            for(size_t index = 0; index < inlierMatches.size(); index++) {
                auto& qid                     = inlierMatches[index].queryIdx;
                auto& tid                     = inlierMatches[index].trainIdx;
                auto  inlier_trans_point_tar  = trans(normalized_target_kpts_pt[tid], translation1, translation2, dtheta);
                inlier_dist_sum              += cv::norm(ref_feature.kpts[qid].pt - inlier_trans_point_tar);
            }
            if(inlier_dist_sum < inlier_min_dist_sum) {
                inlier_min_dist_sum  = inlier_dist_sum;
                inlier_result_dtheta = dtheta;
            }
        }

        if(min_dist_thr < inlier_min_dist_sum / inlierMatches.size()) {
            return false;
        }
        else {
            result_.x     = inlier_result_dpt.x;
            result_.y     = inlier_result_dpt.y;
            result_.theta = inlier_result_dtheta;
            return true;
        }
    }

    const AKAZEPatternMatcher::Delta& AKAZEPatternMatcher::getResult() const {
        return result_;
    }

    cv::Point2f AKAZEPatternMatcher::trans(const cv::Point2f& pt,
                                           const cv::Point2f& center_pt,
                                           const cv::Point2f& dpt,
                                           const double& dtheta) const {
        auto rotate       = Eigen::Rotation2Df(dtheta);
        auto translation1 = Eigen::Translation<float, 2>(center_pt.x + dpt.x, center_pt.y + dpt.y);
        auto translation2 = Eigen::Translation<float, 2>(-center_pt.x, -center_pt.y);
        auto mat          = translation2 * rotate * translation1;

        Eigen::Vector2f projection = mat * Eigen::Vector2f(pt.x, pt.y);
        return cv::Point2f(projection.x(), projection.y());
    }

    cv::Point2f AKAZEPatternMatcher::trans(const cv::Point2f&                  pt,
                                           const Eigen::Translation<float, 2>& trans1,
                                           const Eigen::Translation<float, 2>& trans2,
                                           const double&                       dtheta) const {
        auto rotate = Eigen::Rotation2Df(dtheta);
        auto mat    = trans2 * rotate * trans1;
        Eigen::Vector2f projection = mat * Eigen::Vector2f(pt.x, pt.y);
        return cv::Point2f(projection.x(), projection.y());
    }
}
