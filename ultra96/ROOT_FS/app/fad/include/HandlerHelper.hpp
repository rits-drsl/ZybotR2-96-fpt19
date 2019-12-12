/**
 *  HandlerHelper: fad::Handlerで使用するユーティリティクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_HANDLERHELPER_HPP_
#define INCLUDE_HANDLERHELPER_HPP_

#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <EnvironmentMap.hpp>
#include <VehicleState.hpp>
#include <VisualOdometry.hpp>
#include <RoadType.hpp>
#include <ConstraintType.hpp>
#include <Util.hpp>
#include <ParticleState.hpp>
#include <RouteInfo.hpp>
#include <TrafficLightStatus.hpp>
#include <PedestrianStatus.hpp>

namespace core {
    class HandlerHelper {
    public:
        /**
         *  現状態を表す地図画像を生成する
         */
        static cv::Mat createCurrentWorld(const double&                     side_length,
                                          const EnvironmentMap<RoadType>&   world,
                                          const RouteInfo&                  route_info,
                                          const VehicleState&               tracer_ref_state,
                                          const std::vector<VehicleState>&  path,
                                          const std::vector<VehicleState>&  trajectory,
                                          const std::vector<ParticleState>& particles,
                                          const VisualOdometry&             vo,
                                          const core::TrafficLightStatus&   tl_detection_result,
                                          const core::PedestrianStatus&     pedestrian,
                                          const cv::Mat&                    webcam_img,
                                          const EnvironmentMap<uint8_t>&    road_bin_img,
                                          const EnvironmentMap<uint8_t>&    road_edge_img,
                                          const EnvironmentMap<uint8_t>&    road_sat_img) {
            const cv::Vec3b COLOR_ENTERABLE(192, 168, 168);
            const cv::Vec3b COLOR_OPPOSITE(168, 192, 168);
            const cv::Vec3b COLOR_NOENTRY(64, 64, 64);
            const cv::Vec3b COLOR_PP_EDGE(32, 224, 32);
            const cv::Vec3b COLOR_REF_LINE(32, 8, 32);
            const cv::Vec3b COLOR_REF_EDGE(8, 32, 32);
            const cv::Vec3b COLOR_REAL_BIN_IMG_EDGE(16, 16, 192);
            const cv::Vec3b COLOR_VO_EDGE(192, 16, 16);
            const cv::Vec3b COLOR_TRAJECTORY(16, 192, 192);
            const cv::Vec3b COLOR_TRAJECTORY_LINE(32, 144, 144);
            const cv::Vec3b COLOR_GENERATED_PATH(192, 192, 16);
            const cv::Vec3b COLOR_GENERATED_PATH_LINE(144, 144, 32);
            const cv::Vec3b COLOR_CURRENT_STATE(224, 16, 224);
            const cv::Vec3b COLOR_CURRENT_STATE_TXT(224, 16, 224);
            const cv::Vec3b COLOR_TARGET_STATE(224, 224, 16);
            const cv::Vec3b COLOR_TARGET_STATE_TXT(224, 224, 16);
            const cv::Vec3b COLOR_TRACER_REF_STATE(224, 64, 224);
            const cv::Vec3b COLOR_PARTICLES(32, 255, 128);
            const cv::Vec3b COLOR_ROAD_BIN_IMG(192, 192, 192);
            const cv::Vec3b COLOR_ROAD_EDGE_IMG(16, 224, 16);
            const cv::Vec3b COLOR_ROAD_SAT_IMG(16, 16, 224);
            const cv::Vec3b COLOR_TL_GREEN(32, 255, 32);
            const cv::Vec3b COLOR_TL_RED(32, 32, 255);
            const cv::Vec3b COLOR_TL_TURNING_RED(32, 255, 255);
            const cv::Vec3b COLOR_TL_NO_DETECT(224, 224, 224);
            const cv::Vec3b COLOR_PEDESTRIAN_RECT(128, 255, 32);
            const cv::Vec3b COLOR_PEDESTRIAN_REGION(128, 32, 16);
            const cv::Vec3b COLOR_PEDESTRIAN_NO_DETECT(224, 224, 224);

            const auto med_state           = core::VehicleState((route_info.current_state + route_info.target_states.back()) / 2);
            const auto center_pt           = world.getPixPoint(med_state.x, med_state.y);
            const auto side_length_pix_num = world.getCorrespondPixNum(side_length);
            const auto colored_map_size    = cv::Size(side_length_pix_num, side_length_pix_num);

            cv::Mat colored_map = cv::Mat::zeros(colored_map_size, CV_8UC3);

            // 参照地図・オンライン地図・制約を描画
            for(int yi = -colored_map.rows / 2; yi < colored_map.rows / 2; yi++) {
                for(int xi = -colored_map.cols / 2; xi < colored_map.cols / 2; xi++) {
                    cv::Vec3b outval(0, 0, 0);
                    const auto w_xi = Util::clamp(center_pt.x + xi, 0, world.map.cols - 1);
                    const auto w_yi = Util::clamp(center_pt.y + yi, 0, world.map.rows - 1);

                    if(world.map.at<uint8_t>(w_yi, w_xi) & (uint8_t)RoadType::LINE) {
                        outval += COLOR_REF_LINE;
                    }

                    if(world.map.at<uint8_t>(w_yi, w_xi) & (uint8_t)RoadType::EDGE) {
                        outval += COLOR_REF_EDGE;
                    }

                    const auto r_xi = w_xi - (int)route_info.constraint.getCorrespondPixNum(route_info.region.x);
                    const auto r_yi = w_yi - (int)route_info.constraint.getCorrespondPixNum(route_info.region.y);
                    if(0 <= r_xi && r_xi < route_info.constraint.map.cols && 0 <= r_yi && r_yi < route_info.constraint.map.rows) {
                        if(route_info.constraint.map.at<uint8_t>(r_yi, r_xi) & (uint8_t)ConstraintType::ENTERABLE) {
                            outval += COLOR_ENTERABLE;
                        }
                        else if(route_info.constraint.map.at<uint8_t>(r_yi, r_xi) & (uint8_t)ConstraintType::OPPOSITE) {
                            outval += COLOR_OPPOSITE;
                        }
                        else if(route_info.constraint.map.at<uint8_t>(r_yi, r_xi) & (uint8_t)ConstraintType::NOENTRY) {
                            outval += COLOR_NOENTRY;
                        }
                    }

                    colored_map.at<cv::Vec3b>(yi + colored_map.rows / 2, xi + colored_map.cols / 2) = outval;
                }
            }

            const auto colored_map_center_pt   = cv::Point(colored_map.cols / 2, colored_map.rows / 2);
            const auto current_pt_on_img       = world.getPixPoint(route_info.current_state.x, route_info.current_state.y) - center_pt + colored_map_center_pt;
            const auto road_surface_img_center = cv::Point2d (road_bin_img.map.cols / 2, road_bin_img.map.rows / 2);

            const auto gen_trans_mat_for_road_img = [&](const VehicleState& state) -> cv::Mat {
                                                        const auto add_right_angle = Theta(PI / 2) + state.t;
                                                        const auto corr_offset = cv::Point(world.getCorrespondPixNum(road_bin_img.offset.x) * std::cos(state.t.get()) +
                                                                                     world.getCorrespondPixNum(road_bin_img.offset.y) * std::cos(add_right_angle.get()),
                                                                                     world.getCorrespondPixNum(road_bin_img.offset.x) * std::sin(state.t.get()) +
                                                                                     world.getCorrespondPixNum(road_bin_img.offset.y) * std::sin(add_right_angle.get()));
                                                        cv::Mat trans_mat = cv::getRotationMatrix2D(road_surface_img_center, -add_right_angle.getDegree(), 1.0);
                                                        trans_mat.at<double>(0, 2) += (state.x - route_info.current_state.x) / world.ratio - road_surface_img_center.x + current_pt_on_img.x + corr_offset.x;
                                                        trans_mat.at<double>(1, 2) += (state.y - route_info.current_state.y) / world.ratio - road_surface_img_center.y + current_pt_on_img.y + corr_offset.y;
                                                        return trans_mat;
                                                    };

            // パスプランニングの対象領域を描画
            const auto region_rect_pt = cv::Rect(route_info.constraint.getPixPoint(route_info.region.tl().x, route_info.region.tl().y),
                                                 route_info.constraint.getPixPoint(route_info.region.br().x, route_info.region.br().y));
            cv::rectangle(colored_map, region_rect_pt - center_pt + colored_map_center_pt, COLOR_PP_EDGE, 1, cv::LINE_AA);

            // 取得している路面画像を描画
            cv::Mat trans_mat = gen_trans_mat_for_road_img(route_info.current_state);
            for(int yi = 0; yi < road_bin_img.map.rows; yi++) {
                for(int xi = 0; xi < road_bin_img.map.cols; xi++) {
                    int transed_xi = std::round(trans_mat.at<double>(0, 0) * xi + trans_mat.at<double>(0, 1) * yi + trans_mat.at<double>(0, 2));
                    int transed_yi = std::round(trans_mat.at<double>(1, 0) * xi + trans_mat.at<double>(1, 1) * yi + trans_mat.at<double>(1, 2));
                    auto& pix = colored_map.at<cv::Vec3b>(Util::clamp(transed_yi, 0, colored_map.rows - 1), Util::clamp(transed_xi, 0, colored_map.cols - 1));
                    if(xi == 0 || xi == road_bin_img.map.cols - 1 || yi == 0 || yi == road_bin_img.map.rows - 1) {
                        pix += COLOR_REAL_BIN_IMG_EDGE;
                    }
                }
            }

            // VOの結果を描画
            if(!vo.isZero()) {
                trans_mat = gen_trans_mat_for_road_img(VehicleState(route_info.current_state + vo));
                for(int yi = 0; yi < road_bin_img.map.rows; yi++) {
                    for(int xi = 0; xi < road_bin_img.map.cols; xi++) {
                        if(!(xi == 0 || xi == road_bin_img.map.cols - 1 || yi == 0 || yi == road_bin_img.map.rows - 1)) continue;
                        int transed_xi = std::round(trans_mat.at<double>(0, 0) * xi + trans_mat.at<double>(0, 1) * yi + trans_mat.at<double>(0, 2));
                        int transed_yi = std::round(trans_mat.at<double>(1, 0) * xi + trans_mat.at<double>(1, 1) * yi + trans_mat.at<double>(1, 2));
                        auto& pix = colored_map.at<cv::Vec3b>(Util::clamp(transed_yi, 0, colored_map.rows - 1), Util::clamp(transed_xi, 0, colored_map.cols - 1));
                        pix += COLOR_VO_EDGE;
                    }
                }
            }

            // 軌跡の描画
            std::vector<cv::Point> cv_pt_trajectory;
            std::vector<std::vector<cv::Point>> cv_pt_trajectories;
            for(const auto& t : trajectory) {
                auto pt = world.getPixPoint(t.x, t.y) - center_pt + colored_map_center_pt;

                if(0 <= pt.x && pt.x < colored_map.cols &&
                   0 <= pt.y && pt.y < colored_map.rows) {
                    cv_pt_trajectory.push_back(pt);
                    drawVehicleState(colored_map, pt, t.t, t.v, COLOR_TRAJECTORY, 5);
                }
                else {
                    if(cv_pt_trajectory.size() != 0) {
                        cv_pt_trajectories.push_back(cv_pt_trajectory);
                        cv_pt_trajectory.clear();
                    }
                }
            }
            cv_pt_trajectories.push_back(cv_pt_trajectory);

            for(const auto& cv_pt_trajectory : cv_pt_trajectories) {
                cv::polylines(colored_map, cv_pt_trajectory, false, COLOR_TRAJECTORY_LINE, 1, cv::LINE_AA);
            }

            // 粒子の描画
            for(size_t pi = 0; pi < particles.size(); pi += particles.size() / std::min((int)particles.size(), 100)) {
                auto pt = world.getPixPoint(particles[pi].x, particles[pi].y) - center_pt + colored_map_center_pt;
                cv::circle(colored_map, pt, 1, COLOR_PARTICLES, 1, CV_AA);
            }

            // 生成したパスの描画
            std::vector<cv::Point> cv_pt_path;
            std::vector<std::vector<cv::Point>> cv_pt_paths;
            for(const auto& p : path) {
                auto pt = world.getPixPoint(p.x, p.y) - center_pt + colored_map_center_pt;
                if (0 <= pt.x && pt.x < colored_map.cols && 0 <= pt.y && pt.y < colored_map.rows) {
                    cv_pt_path.push_back(pt);
                    drawVehicleState(colored_map, pt, p.t, p.v, COLOR_GENERATED_PATH, 5);
                }
                else {
                    if(cv_pt_path.size() != 0) {
                        cv_pt_paths.push_back(cv_pt_path);
                        cv_pt_path.clear();
                    }
                }
            }
            cv_pt_paths.push_back(cv_pt_path);

            for(const auto& cv_pt_path : cv_pt_paths) {
                cv::polylines(colored_map, cv_pt_path, false, COLOR_GENERATED_PATH_LINE, 1, cv::LINE_AA);
            }

            // 現状態の描画
            drawVehicleState(colored_map, current_pt_on_img, route_info.current_state.t, route_info.current_state.v, COLOR_CURRENT_STATE, 10);
            const std::string current_pt_str = "(" + std::to_string(route_info.current_state.x) + ", " + std::to_string(route_info.current_state.y) + ")";
            cv::putText(colored_map, current_pt_str, current_pt_on_img + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_CURRENT_STATE_TXT, 1, CV_AA);

            // 目標状態の描画
            for(const auto& target_state : route_info.target_states) {
                const auto target_pt_on_img  = world.getPixPoint(target_state.x, target_state.y) - center_pt + colored_map_center_pt;
                drawVehicleState(colored_map, target_pt_on_img, target_state.t, target_state.v, COLOR_TARGET_STATE, 10);
                const std::string target_pt_str = "(" + std::to_string(target_state.x) + ", " + std::to_string(target_state.y) + ")";
                cv::putText(colored_map, target_pt_str, target_pt_on_img + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TARGET_STATE_TXT, 1, CV_AA);
            }

            // パストレーサの参照状態の描画
            const auto tracer_ref_on_img = world.getPixPoint(tracer_ref_state.x, tracer_ref_state.y) - center_pt + colored_map_center_pt;
            drawVehicleState(colored_map, tracer_ref_on_img, tracer_ref_state.t, tracer_ref_state.v, COLOR_TRACER_REF_STATE, 5);

            // 路面の俯瞰画像を着色
            cv::Mat colored_road;
            cv::cvtColor(road_bin_img.map, colored_road, cv::COLOR_GRAY2BGR);
            for(int yi = 0; yi < colored_road.rows; yi++) {
                for(int xi = 0; xi < colored_road.cols; xi++) {
                    auto val = cv::Vec3b(0, 0, 0);
                    if(colored_road.at<cv::Vec3b>(yi, xi) != cv::Vec3b(0, 0, 0)) {
                        val += COLOR_ROAD_BIN_IMG;
                    }
                    if(road_edge_img.map.at<uint8_t>(yi, xi) != 0) {
                        val += COLOR_ROAD_EDGE_IMG;
                    }
                    if(road_sat_img.map.at<uint8_t>(yi, xi) != 0) {
                        val += COLOR_ROAD_SAT_IMG;
                    }
                    colored_road.at<cv::Vec3b>(yi, xi) = val;
                }
            }

            // Webカメラの画像の横幅を路面の俯瞰画像の横幅に合わせる
            cv::Mat scaled_webcam_img;
            const double webcam_scale_ratio = colored_road.cols / (double)webcam_img.cols;
            cv::resize(webcam_img, scaled_webcam_img, cv::Size(), webcam_scale_ratio, webcam_scale_ratio, cv::INTER_CUBIC);

            // 信号・歩行者認識の結果を描画する
            auto draw_detection_result = [&](const cv::Rect& rect, const std::string& comment, const cv::Vec3b& color) -> void {
                                             if(!rect.empty()) {
                                                 cv::rectangle(scaled_webcam_img, rect, color, 1, cv::LINE_AA);
                                                 cv::putText(scaled_webcam_img, comment, cv::Point(rect.x, rect.y - 5),
                                                             cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1, CV_AA);
                                             }
                                         };
            const auto scaled_tl_rect = cv::Rect(tl_detection_result.rect.x * webcam_scale_ratio, tl_detection_result.rect.y * webcam_scale_ratio,
                                                 tl_detection_result.rect.width * webcam_scale_ratio, tl_detection_result.rect.height * webcam_scale_ratio);
            switch(tl_detection_result.type) {
                case core::TrafficLightStatus::Type::GREEN: {
                    draw_detection_result(scaled_tl_rect, tl_detection_result.comment, COLOR_TL_GREEN);
                    cv::putText(scaled_webcam_img, "[TL] Green", cv::Point(0, scaled_webcam_img.rows - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TL_GREEN, 1, CV_AA);
                    break;
                }
                case core::TrafficLightStatus::Type::RED: {
                    draw_detection_result(scaled_tl_rect, tl_detection_result.comment, COLOR_TL_RED);
                    cv::putText(scaled_webcam_img, "[TL] Red", cv::Point(0, scaled_webcam_img.rows - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TL_RED, 1, CV_AA);
                    break;
                }
                case core::TrafficLightStatus::Type::TURNING_RED: {
                    draw_detection_result(scaled_tl_rect, tl_detection_result.comment, COLOR_TL_TURNING_RED);
                    cv::putText(scaled_webcam_img, "[TL] Turning Red", cv::Point(0, scaled_webcam_img.rows - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TL_TURNING_RED, 1, CV_AA);
                    break;
                }
                case core::TrafficLightStatus::Type::NO_DETECT: {
                    draw_detection_result(cv::Rect(), "", COLOR_TL_NO_DETECT);
                    cv::putText(scaled_webcam_img, "[TL] NO Detect", cv::Point(0, scaled_webcam_img.rows - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TL_NO_DETECT, 1, CV_AA);
                    break;
                }
            }

            if(pedestrian.rect.empty()) {
                cv::putText(scaled_webcam_img, "[P] NO Detect", cv::Point(0, 0),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_PEDESTRIAN_NO_DETECT, 1, CV_AA);
            }
            else {
                const auto scaled_pedestrian_rect = cv::Rect(pedestrian.rect.x * webcam_scale_ratio, pedestrian.rect.y * webcam_scale_ratio,
                                                             pedestrian.rect.width * webcam_scale_ratio, pedestrian.rect.height * webcam_scale_ratio);
                draw_detection_result(scaled_pedestrian_rect, pedestrian.comment, COLOR_PEDESTRIAN_RECT);
                for(int yi = 0; yi < pedestrian.region.rows; yi++) {
                    for(int xi = 0; xi < pedestrian.region.cols; xi++) {
                        scaled_webcam_img.at<cv::Vec3b>(yi + scaled_pedestrian_rect.y, xi + scaled_pedestrian_rect.x) +=
                            pedestrian.region.at<uint8_t>(yi, xi) == 0 ? cv::Vec3b(0, 0, 0) : COLOR_PEDESTRIAN_REGION;
                    }
                }
            }

            // 路面の俯瞰画像・Webカメラの画像を地図画像の右側に連結
            cv::Mat combined = cv::Mat::zeros(cv::Size(colored_map.cols + std::max(colored_road.cols, scaled_webcam_img.cols),
                                                       std::max(colored_map.rows, colored_road.rows + scaled_webcam_img.rows)), CV_8UC3);
            cv::Mat left(combined, cv::Rect(0, 0, colored_map.cols, colored_map.rows));
            cv::Mat right_top(combined, cv::Rect(colored_map.cols, 0, colored_road.cols, colored_road.rows));
            cv::Mat right_bottom(combined, cv::Rect(colored_map.cols, colored_road.rows, scaled_webcam_img.cols, scaled_webcam_img.rows));
            colored_map.copyTo(left);
            colored_road.copyTo(right_top);
            scaled_webcam_img.copyTo(right_bottom);

            return combined;
        }

        /**
         *  VehicleStateをcv::Matに描画する
         */
        static void drawVehicleState(const cv::Mat&    img,
                                     const cv::Point&  pt,
                                     const Theta&      theta,
                                     const double&     velocity,
                                     const cv::Scalar& color,
                                     const double&     size) {
            const auto circle_size = std::min(size + std::abs(velocity * 30), size * 2);
            const auto dst_pt = pt + cv::Point(circle_size * 1.5 * std::cos(theta.get()),
                                               circle_size * 1.5 * std::sin(theta.get()));
            cv::circle(img, pt, circle_size, color, 1, CV_AA);
            cv::line(img, pt, dst_pt, color, 1, CV_AA);
        }
    };
}

#endif /* INCLUDE_HANDLERHELPER_HPP_ */
