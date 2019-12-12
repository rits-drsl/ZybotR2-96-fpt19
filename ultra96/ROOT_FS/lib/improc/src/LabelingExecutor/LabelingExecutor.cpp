/**
 *  LabelingExecutor: ラベリングを実行するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *
 */

#include <LabelingExecutor/LabelingExecutor.h>

namespace improc {
    std::map<uint16_t, LabelingExecutor::Info> LabelingExecutor::execute(const cv::Mat& src,
                                                                         const uint8_t& bin_thr) {
        cv::Mat dst;
        return execute(src, dst, bin_thr);
    }

    std::map<uint16_t, LabelingExecutor::Info> LabelingExecutor::execute(const cv::Mat& src,
                                                                         cv::Mat&       dst,
                                                                         const uint8_t& bin_thr) {
        std::map<uint16_t, Info> info_map;
        execute(src, dst, info_map, bin_thr);
        return info_map;
    }

    void LabelingExecutor::execute(const cv::Mat&            src,
                                   cv::Mat&                  dst,
                                   std::map<uint16_t, Info>& label_info_map,
                                   const uint8_t&            bin_thr) {
        label_info_map.clear();
        dst = cv::Mat(src.size(), CV_16UC1);

        // LUT1(ラベルの対応関係と未参照フラグ)
        pair_lut lut_1;

        //--- ラベリング(第一段階)
        uint16_t num_label  = 1;
        uint16_t prev_label = 0;

        [&]() {
            for(int yi = 0; yi < dst.rows; yi++) {
                for(int xi = 0; xi < dst.cols; xi++) {
                    // 条件１. 画素値がしきい値以上
                    if(bin_thr <= src.data[xi + yi * dst.cols]) {
                        int up_label = dst.data[xi + (yi - 1) * dst.cols];

                        // 条件２. 上のラベルが存在する場合
                        if(up_label != 0 && yi != 0) {
                            dst.data[xi + yi * dst.cols] = up_label;

                            // 条件３. 上のラベルと左のラベルが存在する場合
                            if(prev_label != 0 && prev_label != up_label) {
                                // LUT1に対応関係を記録する
                                if(prev_label < up_label) {
                                    lut_1.emplace(std::make_tuple(prev_label, up_label), false);
                                }
                                else {
                                    lut_1.emplace(std::make_tuple(up_label, prev_label), false);
                                }
                            }
                            prev_label = up_label;
                        }
                        // 条件４. 上のラベルが存在せず、左のラベルが存在する場合
                        else if(prev_label != 0) {
                            dst.data[xi + yi * dst.cols] = prev_label;
                        }
                        // 条件５. 上のラベルも左のラベルも存在しない場合
                        else {
                            if(num_label == 0xFFFF) {
                                std::cout << "can not do labeling any more" << std::endl;
                                return;
                            }
                            else {
                                dst.data[xi + yi * dst.cols] = num_label;
                                prev_label = num_label;
                                num_label++;
                            }
                        }
                    }
                    // 条件６. 画素値が0
                    else {
                        dst.data[xi + yi * dst.cols] = 0;
                        prev_label = 0;
                    }
                }
            }
        }();

        //--- LUT1の対応関係を整理し、LUT2を作成する(メモ化再帰)
        std::vector<uint16_t> lut_2;

        // 初期化
        for(int i = 0; i < num_label; i++) {
            lut_2.push_back(i);
        }

        for(const auto& L1 : lut_1) {
            const auto& s_label_number = std::get<SMALLER_LABEL>(L1.first);
            const auto& b_label_number = std::get<BIGGER_LABEL>(L1.first);
            const auto& referenced     = L1.second;

            if(!referenced) {
                const auto key   = b_label_number;
                const auto value = s_label_number;

                lut_1[L1.first] = true;
                lut_2[key]      = value;

                // 連結しているラベルをLUT1から探索し、LUT2に記録する
                solveLUT(lut_1, lut_2, key, value);
            }
        }

        // ラベル番号 -> x方向の投影ヒストグラム
        std::vector<std::vector<uint16_t>> x_hist_map(num_label, std::vector<uint16_t>(dst.cols, 0));

        // ラベル番号 -> 画素値の合計(平均画素値を出すために保持する)
        std::vector<uint32_t> label_brightness_sum(num_label, 0);

        //--- ラベリング(第二段階)
        for(int yi = 0; yi < dst.rows; yi++) {
            for(int xi = 0; xi < dst.cols; xi++) {
                // LUTからラベル番号を得る
                const auto& label_index = lut_2[dst.data[xi + yi * dst.cols]];
                if(label_index != 0) {
                    dst.data[xi + yi * dst.cols] = label_index;
                    auto label_info_itr = label_info_map.find(label_index);

                    // labelがmapに未登録の場合
                    if(label_info_itr == label_info_map.end()) {
                        // 領域情報を保持するpair(first -> 左上の座標、second -> 右下の座標)
                        Info label_info;

                        // 領域情報の初期化
                        label_info.begin.x  = xi;
                        label_info.begin.y  = yi;
                        label_info.end.x    = 0;
                        label_info.end.y    = 0;

                        // 面積情報の初期化
                        label_info.area = 1;

                        // 重心情報の初期化
                        label_info.cog = 0;

                        // 平均輝度値の初期化
                        label_info.brightness = 0;

                        // mapに登録
                        label_info_map.emplace(label_index, label_info);
                    }
                    // labelがmapに登録済みの場合
                    else {
                        // 領域情報の更新
                        label_info_itr->second.begin.x = std::min(label_info_itr->second.begin.x, xi);
                        label_info_itr->second.end.x   = std::max(label_info_itr->second.end.x, xi);
                        label_info_itr->second.end.y   = std::max(label_info_itr->second.end.y, yi);

                        // 面積情報の更新
                        label_info_itr->second.area += 1;
                    }

                    // 投影ヒストグラムの更新
                    x_hist_map[label_index][xi] += 1;

                    // 画素値の合計の更新
                    label_brightness_sum[label_index] += src.data[xi + yi * dst.cols];
                }
            }
        }

        // 各ラベル領域の重心・平均画素値・中心座標を導出
        for(const auto& label_info : label_info_map) {
            auto numerator   = 0;
            auto denominator = 0;
            for(int xhi = 0; xhi < dst.cols; xhi++) {
                numerator   += xhi*x_hist_map[label_info.first][xhi];
                denominator += x_hist_map[label_info.first][xhi];
            }

            auto label_info_map_itr = label_info_map.find(label_info.first);
            label_info_map_itr->second.cog        = numerator / denominator;
            label_info_map_itr->second.brightness = label_brightness_sum[label_info.first] / label_info_map_itr->second.area;
            label_info_map_itr->second.center     = (label_info_map_itr->second.begin + label_info_map_itr->second.end) / 2;
        }
    }

    void LabelingExecutor::solveLUT(pair_lut&              lut_1,
                                    std::vector<uint16_t>& lut_2,
                                    uint16_t               key,
                                    uint16_t               value) {
        // keyに連結しているラベルを保持する可変長配列
        std::vector<uint16_t> cat_label_v;

        // LUT1に対するラベルの線形探索
        for(const auto& L1 : lut_1) {
            const auto& s_label_number = std::get<SMALLER_LABEL>(L1.first);
            const auto& b_label_number = std::get<BIGGER_LABEL>(L1.first);
            const auto& referenced     = L1.second;

            if(!referenced) {
                if(s_label_number == key) {
                    cat_label_v.push_back(b_label_number);
                    lut_1[L1.first] = true;
                }
                else if(b_label_number == key) {
                    cat_label_v.push_back(s_label_number);
                    lut_1[L1.first] = true;
                }
            }

            // ソート済みなので小さい方のラベルがkey以上になったら探索終了
            if(key < s_label_number) {
                break;
            }
        }

        // LUT2に書き込んで再帰
        if(cat_label_v.size() != 0) {
            for(size_t i = 0; i < cat_label_v.size(); i++) {
                lut_2[cat_label_v[i]] = value;
                solveLUT(lut_1, lut_2, cat_label_v[i], value);
            }
        }
        // 探索終了
        else {
            return;
        }
    }
}
