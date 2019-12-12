/**
 *  Handler: 自動運転コンテスト向けのプログラムを実行するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_HANDLER_H_
#define FAD_SRC_FAD_HANDLER_H_

#include <thread>
#include <mutex>

#include <ProcTimer.hpp>
#include <HandlerHelper.hpp>

#include "RoutePlanner/RoutePlanner.h"
#include "StateEstimator/StateEstimator.h"
#include "ConstraintGenerator/ConstraintGenerator.h"
#include "OdometryCalculator/WOCalculator/WOCalculator.h"
#include "OdometryCalculator/IOCalculator/IOCalculator.h"
#include "OdometryCalculator/PatternMatchingVOCalculator/PatternMatchingVOCalculator.h"
#include "PathPlanner/PathPlanner.h"
#include "PathTracer/PathTracer.h"
#include "HWController/CmodController/CmodController.h"
#include "HWController/PcamController/PcamController.h"
#include "HWController/WebcamController/WebcamController.h"
#include "HWController/OptorIMUController/OptorIMUController.h"

namespace fad {
    class Handler {
    public:
        enum class Mode {
            NORMAL,
            DEBUG
        };

        Handler();
        ~Handler();

        Handler(const Handler& obj) = delete;
        Handler &operator=(const Handler& obj) = delete;

        /**
         *  自動運転プログラムを実行する
         *  @mode: 実行モード
         *         Mode::RUN   ... デバッグ出力なし
         *         Mode::DEBUG ... デバッグ出力あり
         */
        void run(const Mode& mode = Mode::NORMAL);
    private:
        const std::string HANDLER_PARAM_YAML_PATH = "/data/Handler/param.yaml";

        class HandlerParam : public core::YAMLHelper::ParamBase {
        public:
            double      terminate_distance;
            std::string optor_dev_filename;

            void read(const cv::FileNode& node) override {
                terminate_distance = (double)node["terminate_distance"];
                optor_dev_filename = (std::string)node["optor_dev_filename"];
            }
        };

        class DebugParam : public core::YAMLHelper::ParamBase {
        public:
            double   img_side_length;
            uint32_t interval_imshow_debug_map;
            bool     imwrite_mode;

            void read(const cv::FileNode& node) override {
                img_side_length           = (double)node["img_side_length"];
                interval_imshow_debug_map = (int)node["interval_imshow_debug_map"];
                imwrite_mode              = (int)node["imwrite_mode"];
            }
        };

        class StartupDelay : public core::YAMLHelper::ParamBase {
        public:
            double wo_calculating;
            double io_calculating;
            double vo_calculating;
            double localizationing;
            double planning;
            double tracing;
            double debagging;

            void read(const cv::FileNode& node) override {
                wo_calculating  = (double)node["wo_calculating"];
                io_calculating  = (double)node["io_calculating"];
                vo_calculating  = (double)node["vo_calculating"];
                localizationing = (double)node["localizationing"];
                planning        = (double)node["planning"];
                tracing         = (double)node["tracing"];
                debagging       = (double)node["debagging"];
            }
        };

        class ExecutePeriod : public core::YAMLHelper::ParamBase {
        public:
            double wo_calculating;
            double io_calculating;
            double vo_calculating;
            double localizationing;
            double planning;
            double tracing;
            double debagging;

            void read(const cv::FileNode& node) override {
                wo_calculating  = (double)node["wo_calculating"];
                io_calculating  = (double)node["io_calculating"];
                vo_calculating  = (double)node["vo_calculating"];
                localizationing = (double)node["localizationing"];
                planning        = (double)node["planning"];
                tracing         = (double)node["tracing"];
                debagging       = (double)node["debagging"];
            }
        };

        struct AverageTime {
            double th_wo_calculating{0};
            double th_io_calculating{0};
            double th_vo_calculating{0};
            double th_localizationing{0};
            double th_planning{0};
            double th_tracing{0};
            double th_debagging{0};
            AverageTime() = default;
        };

        template<typename T>
        class ExclusiveObj {
        public:
            T obj;
            void lock() { mtx_.lock(); }
            bool try_lock() { return mtx_.try_lock(); }
            void unlock() { mtx_.unlock(); }
        private:
            std::mutex mtx_;
        };

        const bool x11_is_valid_;

        bool optor_can_open_;

        ExclusiveObj<std::unique_ptr<RoutePlanner>>                route_planner_;
        ExclusiveObj<std::unique_ptr<StateEstimator>>              state_estimator_;
        ExclusiveObj<std::unique_ptr<ConstraintGenerator>>         constraint_generator_;
        ExclusiveObj<std::unique_ptr<WOCalculator>>                wo_calc_;
        ExclusiveObj<std::unique_ptr<IOCalculator>>                io_calc_;
        ExclusiveObj<std::unique_ptr<PatternMatchingVOCalculator>> vo_calc_;
        ExclusiveObj<std::unique_ptr<PathPlanner>>                 path_planner_;
        ExclusiveObj<std::unique_ptr<PathTracer>>                  path_tracer_;
        ExclusiveObj<std::unique_ptr<CmodController>>              cmod_ctrl_;
        ExclusiveObj<std::unique_ptr<PcamController>>              pcam_ctrl_;
        ExclusiveObj<std::unique_ptr<WebcamController>>            webcam_ctrl_;
        ExclusiveObj<std::unique_ptr<OptorIMUController>>          optor_imu_ctrl_;

        HandlerParam  param_;
        DebugParam    debug_param_;
        ExecutePeriod exec_period_;
        StartupDelay  startup_delay_;

        ExclusiveObj<bool>               run_flag_;
        ExclusiveObj<core::VehicleState> current_state_;
        ExclusiveObj<AverageTime>        average_time_;

        void init();

        void doWOCalculatingWorker(const bool& debug_mode = false);
        void doIOCalculatingWorker(const bool& debug_mode = false);
        void doVOCalculatingWorker(const bool& debug_mode = false);
        void doLocalizationingWorker(const bool& debug_mode = false);
        void doPlanningWorker(const bool& debug_mode = false);
        void doTracingWorker(const bool& debug_mode = false);
        void doDebuggingWorker(const bool& debug_mode = false);
    };
}

#endif /* FAD_SRC_FAD_HANDLER_H_ */
