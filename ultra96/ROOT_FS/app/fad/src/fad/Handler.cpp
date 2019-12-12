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

#include "Handler.h"

namespace fad {
    Handler::Handler() :
        x11_is_valid_(std::getenv("DISPLAY") != nullptr ? true : false)
    {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        // 処理に関するパラメータを読み込む
        core::YAMLHelper::readStruct(root_path + HANDLER_PARAM_YAML_PATH, param_, "Basis");

        // 各スレッドの立ち上がり時の待機時間を読み込む
        core::YAMLHelper::readStruct(root_path + HANDLER_PARAM_YAML_PATH, startup_delay_, "StartupDelay");

        // 各処理の実行周期を読み込む
        core::YAMLHelper::readStruct(root_path + HANDLER_PARAM_YAML_PATH, exec_period_, "ExecutePeriod");

        // デバッグに使用するパラメータを読み込む
        core::YAMLHelper::readStruct(root_path + HANDLER_PARAM_YAML_PATH, debug_param_, "Debug");

        // Optorが使用可能か確認する
        std::ifstream fs(param_.optor_dev_filename);
        optor_can_open_ = fs.is_open();

        // 各インスタンスを生成する
        route_planner_.obj        = std::make_unique<RoutePlanner>();
        state_estimator_.obj      = std::make_unique<StateEstimator>();
        constraint_generator_.obj = std::make_unique<ConstraintGenerator>();
        path_planner_.obj         = std::make_unique<PathPlanner>();
        path_tracer_.obj          = std::make_unique<PathTracer>();
        cmod_ctrl_.obj            = std::make_unique<CmodController>();
        pcam_ctrl_.obj            = std::make_unique<PcamController>();
        webcam_ctrl_.obj          = std::make_unique<WebcamController>();
        wo_calc_.obj              = std::make_unique<WOCalculator>();
        vo_calc_.obj              = std::make_unique<PatternMatchingVOCalculator>();
        io_calc_.obj              = std::make_unique<IOCalculator>();
    }

    Handler::~Handler() {
    }

    void Handler::run(const Mode& mode) {
        std::exception_ptr ep;
        auto gen_th_lambda = [&](auto do_worker) -> auto {
                                 return [&]() -> void {
                                            try {
                                                do_worker();
                                            }
                                            catch(...) {
                                                std::lock_guard<ExclusiveObj<bool>> lock(run_flag_);
                                                run_flag_.obj = false;
                                                ep = std::current_exception();
                                            }
                                        };
                             };

        // CmodのBTN2の押下を待機
        std::cout << "It is initialized when the Cmod BTN2 is pressed" << std::endl;
        while(true && !ep) {
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<CmodController>>> lock_cmod(cmod_ctrl_);
                if(cmod_ctrl_.obj->BTNIsPushed(CmodController::BTN::CMOD2)) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 初期化
        init();

        // PathTracer以外のスレッドを起動
        bool debug_mode = (mode == Mode::DEBUG) ? true : false;
        auto th_wo_calculating  = std::thread(gen_th_lambda([&]() -> void { doWOCalculatingWorker(debug_mode); }));
        auto th_io_calculating  = std::thread(gen_th_lambda([&]() -> void { doIOCalculatingWorker(debug_mode); }));
        auto th_vo_calculating  = std::thread(gen_th_lambda([&]() -> void { doVOCalculatingWorker(debug_mode); }));
        auto th_localizationing = std::thread(gen_th_lambda([&]() -> void { doLocalizationingWorker(debug_mode); }));
        auto th_planning        = std::thread(gen_th_lambda([&]() -> void { doPlanningWorker(debug_mode); }));
        auto th_debagging       = std::thread(gen_th_lambda([&]() -> void { doDebuggingWorker(debug_mode); }));

        // CmodのBTN1の押下を待機
        std::cout << "It is started when the Cmod BTN1 is pressed"    << std::endl;
        std::cout << "At emergency stop, press the Cmod BTN2 to stop" << std::endl;
        while(true && !ep) {
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<CmodController>>> lock_cmod(cmod_ctrl_);
                if(cmod_ctrl_.obj->BTNIsPushed(CmodController::BTN::CMOD1)) break;
            }
            {
                std::lock_guard<ExclusiveObj<bool>> lock(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // PathTracerを起動
        auto th_tracing = std::thread(gen_th_lambda([&]() -> void { doTracingWorker(debug_mode); }));

        // 終了を待機
        th_wo_calculating.join();
        th_io_calculating.join();
        th_vo_calculating.join();
        th_localizationing.join();
        th_planning.join();
        th_tracing.join();
        th_debagging.join();

        cmod_ctrl_.obj->setWheelAngularVelocity(0.0, 0.0);

        if(ep) {
            std::rethrow_exception(ep);
        }
    }

    void Handler::init() {
        route_planner_.obj->init();
        constraint_generator_.obj->init();
        run_flag_.obj = true;
        current_state_.obj = route_planner_.obj->getRefCurrentState();
        state_estimator_.obj->init(current_state_.obj);
        vo_calc_.obj->init(route_planner_.obj->getWorldMap());
    }

    void Handler::doWOCalculatingWorker(const bool& debug_mode) {
        core::ProcTimer          timer(10);
        CmodController::MotorCnt cnt;

        {
            std::lock_guard<ExclusiveObj<std::unique_ptr<CmodController>>> lock_cmod(cmod_ctrl_);
            cnt = cmod_ctrl_.obj->getCntOfRotaryEncoder();

            std::lock_guard<ExclusiveObj<std::unique_ptr<WOCalculator>>> lock_wo(wo_calc_);
            wo_calc_.obj->init(cnt.right, cnt.left);
        }

        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.wo_calculating * 1e6)));
        while(true) {
            timer.start();
            // ロータリーエンコーダのカウンタ値を取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<CmodController>>> lock(cmod_ctrl_);
                cnt = cmod_ctrl_.obj->getCntOfRotaryEncoder();
            }

            // wheel odometryの状態を更新
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<WOCalculator>>> lock(wo_calc_);
                wo_calc_.obj->update(cnt.right, cnt.left);
            }

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock_avt(average_time_);
                average_time_.obj.th_wo_calculating = timer.getAverageElapsedTime();

                std::lock_guard<ExclusiveObj<bool>> lock_rf(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.wo_calculating * 1e6)));
        }
    }

    void Handler::doIOCalculatingWorker(const bool& debug_mode) {
        if(!optor_can_open_) return;

        core::ProcTimer         timer(10);
        core::WheelOdometry     total_wo, prev_total_wo;
        core::VehicleState      current_state;
        fad::OptorIMUController optor_ctrl;

        optor_ctrl.update();
        {
            std::lock_guard<ExclusiveObj<std::unique_ptr<IOCalculator>>> lock_io(io_calc_);
            io_calc_.obj->init(optor_ctrl.getQuat());
        }

        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.io_calculating * 1e6)));
        while(true) {
            timer.start();
            {
                std::lock_guard<ExclusiveObj<core::VehicleState>> lock_state(current_state_);
                current_state = current_state_.obj;
            }

            // WOを取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<WOCalculator>>> lock_io(wo_calc_);
                total_wo = wo_calc_.obj->getTotal(current_state.t);
            }

            // inertial odometryの状態を更新
            optor_ctrl.update();
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<IOCalculator>>> lock_io(io_calc_);
                io_calc_.obj->update(optor_ctrl.getQuat(), total_wo.distanceFrom(prev_total_wo));
            }
            prev_total_wo = total_wo;

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock_avt(average_time_);
                average_time_.obj.th_io_calculating = timer.getAverageElapsedTime();

                std::lock_guard<ExclusiveObj<bool>> lock_rf(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.io_calculating * 1e6)));
        }
    }

    void Handler::doVOCalculatingWorker(const bool& debug_mode) {
        core::ProcTimer               timer(10);
        double                        world_ratio;
        core::VehicleState            current_state;
        core::EnvironmentMap<uint8_t> bin_img, edge_img;

        {
            std::lock_guard<ExclusiveObj<std::unique_ptr<RoutePlanner>>> lock(route_planner_);
            world_ratio = route_planner_.obj->getWorldMap().ratio;
        }

        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.vo_calculating * 1e6)));
        while(true) {
            timer.start();
            {
                std::lock_guard<ExclusiveObj<core::VehicleState>> lock_state(current_state_);
                current_state = current_state_.obj;
            }

            // 路面の俯瞰画像を更新・取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<PcamController>>> lock(pcam_ctrl_);
                pcam_ctrl_.obj->updateRoadSurfaceImage(world_ratio);
                bin_img  = pcam_ctrl_.obj->getRoadSurfaceImage(PcamController::ImageType::BIN).clone();
                edge_img = pcam_ctrl_.obj->getRoadSurfaceImage(PcamController::ImageType::EDGE).clone();
            }

            // 参照地図との変位を導出する
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<PatternMatchingVOCalculator>>> lock(vo_calc_);
                vo_calc_.obj->update(current_state, bin_img, edge_img);
            }

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock_avt(average_time_);
                average_time_.obj.th_vo_calculating = timer.getAverageElapsedTime();

                std::lock_guard<ExclusiveObj<bool>> lock_rf(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.vo_calculating * 1e6)));
        }
    }

    void Handler::doLocalizationingWorker(const bool& debug_mode) {
        core::ProcTimer        timer(10);
        core::VehicleState     current_state;
        core::WheelOdometry    wo;
        core::InertialOdometry io;
        core::VisualOdometry   vo;

        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.localizationing * 1e6)));
        while(true) {
            timer.start();
            {
                std::lock_guard<ExclusiveObj<core::VehicleState>> lock_state(current_state_);
                current_state = current_state_.obj;
            }

            // wheel odometryによって観測された変位を取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<WOCalculator>>> lock(wo_calc_);
                wo = wo_calc_.obj->get(current_state.t);
            }

            // inertial odometryによって観測された変位を取得
            // Optorが使用可能でなければWOを使用する
            if(optor_can_open_) {
                std::lock_guard<ExclusiveObj<std::unique_ptr<IOCalculator>>> lock(io_calc_);
                io = io_calc_.obj->get(current_state.t);
            }
            else {
                io = core::InertialOdometry(wo.x, wo.y, wo.t);
            }

            // 路面の俯瞰画像と参照地図のマッチングによって得られた補正値を取得
            // NOTE: PatternMatchingVOCalculator::update()の処理時間が長いため、
            //       mutexがロックされていない場合のみ値を取得する
            {
                std::unique_lock<ExclusiveObj<std::unique_ptr<PatternMatchingVOCalculator>>> lock(vo_calc_, std::try_to_lock);
                if(lock.owns_lock()) {
                    vo = vo_calc_.obj->get();
                }
                else {
                    vo = core::VisualOdometry();
                }
            }

            // 自己位置を推定・現在位置を設定
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<StateEstimator>>> lock_est(state_estimator_);
                state_estimator_.obj->update(wo, io, vo);

                std::lock_guard<ExclusiveObj<core::VehicleState>> lock_state(current_state_);
                current_state_.obj = state_estimator_.obj->getCurrentState();

                std::lock_guard<ExclusiveObj<std::unique_ptr<RoutePlanner>>> lock(route_planner_);
                route_planner_.obj->setCurrentState(state_estimator_.obj->getCurrentState());

                std::lock_guard<ExclusiveObj<std::unique_ptr<PathTracer>>> lock_tracer(path_tracer_);
                path_tracer_.obj->setCurrentState(state_estimator_.obj->getCurrentState());
            }

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock_avt(average_time_);
                average_time_.obj.th_localizationing = timer.getAverageElapsedTime();

                std::lock_guard<ExclusiveObj<bool>> lock_rf(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.localizationing * 1e6)));
        }
    }

    void Handler::doPlanningWorker(const bool& debug_mode) {
        core::ProcTimer                timer(10);
        double                         world_ratio;
        std::unique_ptr<core::TLState> tl_state_ptr;
        core::EnvironmentMap<uint8_t>  sat_img;
        cv::Mat                        color_img;
        core::RouteInfo                route_info;
        core::VehicleState             current_state;

        {
            std::lock_guard<ExclusiveObj<std::unique_ptr<RoutePlanner>>> lock(route_planner_);
            world_ratio = route_planner_.obj->getWorldMap().ratio;
        }

        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.planning * 1e6)));
        while(true) {
            timer.start();
            {
                std::lock_guard<ExclusiveObj<core::VehicleState>> lock(current_state_);
                current_state = current_state_.obj;
            }

            // 経路生成に必要となる情報を取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<RoutePlanner>>> lock(route_planner_);
                route_info = route_planner_.obj->generateRouteInfomation();

                core::TLState tl_state;
                if(route_planner_.obj->searchStateOfTL(tl_state)) {
                    tl_state_ptr = std::make_unique<core::TLState>(std::move(tl_state));
                }
                else {
                    tl_state_ptr = nullptr;
                }

                // 終了状態に到達していた場合、処理を終了する
                const auto dist_cur2tar = route_info.target_states.back().distanceFrom(route_info.current_state);
                if(route_planner_.obj->reachedGoal() && dist_cur2tar < param_.terminate_distance) {
                    std::lock_guard<ExclusiveObj<bool>> lock(run_flag_);
                    run_flag_.obj = false;
                }
            }

            // 路面の俯瞰画像を取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<PcamController>>> lock(pcam_ctrl_);
                pcam_ctrl_.obj->updateRoadSurfaceImage(world_ratio);
                sat_img = pcam_ctrl_.obj->getRoadSurfaceImage(PcamController::ImageType::SAT).clone();
            }

            // 前方の画像を取得
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<WebcamController>>> lock(webcam_ctrl_);
                webcam_ctrl_.obj->updateWebCamImage();
                color_img = webcam_ctrl_.obj->getWebCamImage().clone();
            }

            // 経路生成に必要となる制約を生成・パスを生成
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<ConstraintGenerator>>> lock_cg(constraint_generator_);
                if(tl_state_ptr != nullptr) {
                    constraint_generator_.obj->detectTrafficLights(color_img, current_state.distanceFrom(*tl_state_ptr.get()));
                }
                constraint_generator_.obj->detectObstacles(sat_img, current_state);
                constraint_generator_.obj->detectPedestrians(color_img, current_state);
                constraint_generator_.obj->generate(route_info);
                route_info = constraint_generator_.obj->getRouteInfo();
            }

            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<PathPlanner>>> lock_pp(path_planner_);
                path_planner_.obj->applyConstraint(route_info.constraint, route_info.region);
                const auto status = path_planner_.obj->solve(route_info.current_state, route_info.target_states);
                if(status) {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<PathTracer>>> lock_pt(path_tracer_);
                    path_tracer_.obj->setPath(path_planner_.obj->getResult());
                }
            }

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock_avt(average_time_);
                average_time_.obj.th_planning = timer.getAverageElapsedTime();

                std::lock_guard<ExclusiveObj<bool>> lock_rf(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.planning * 1e6)));
        }
    }

    void Handler::doTracingWorker(const bool& debug_mode) {
        core::ProcTimer timer(10);

        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.tracing * 1e6)));
        while(true) {
            timer.start();
            // パスを追従する角速度を導出・設定
            {
                std::lock_guard<ExclusiveObj<std::unique_ptr<PathTracer>>> lock_tracer(path_tracer_);
                auto velocity = path_tracer_.obj->calcAngularVelocity();

                std::lock_guard<ExclusiveObj<std::unique_ptr<CmodController>>> lock_cmod(cmod_ctrl_);
                cmod_ctrl_.obj->setWheelAngularVelocity(velocity.r, velocity.l);

                // Cmod BTN2が押されている場合、強制終了する
                if(cmod_ctrl_.obj->BTNIsPushed(CmodController::BTN::CMOD2)) {
                    std::lock_guard<ExclusiveObj<bool>> lock(run_flag_);
                    run_flag_.obj = false;
                }
            }

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock_avt(average_time_);
                average_time_.obj.th_tracing = timer.getAverageElapsedTime();

                std::lock_guard<ExclusiveObj<bool>> lock_rf(run_flag_);
                if(!run_flag_.obj) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.tracing * 1e6)));
        }
    }

    void Handler::doDebuggingWorker(const bool& debug_mode) {
        if(!debug_mode) return;

        size_t                               index = 0;
        core::ProcTimer                      timer(10);
        core::VehicleState                   ref_state;
        core::EnvironmentMap<core::RoadType> world;
        core::EnvironmentMap<uint8_t>        bin_img, edge_img, sat_img;
        cv::Mat                              webcam_img;
        core::RouteInfo                      route_info;
        core::TrafficLightStatus             tl_detection_result;
        core::PedestrianStatus               pedestrian;
        core::VisualOdometry                 vo;
        std::vector<core::VehicleState>      path, trajectory;
        std::vector<core::ParticleState>     particles;
        AverageTime                          average_time;

        {
            std::lock_guard<ExclusiveObj<std::unique_ptr<RoutePlanner>>> lock(route_planner_);
            world = route_planner_.obj->getWorldMap().clone();
        }

        cv::namedWindow("Online Map", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
        std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(startup_delay_.debagging * 1e6)));
        while(true) {
            timer.start();
            {
                std::lock_guard<ExclusiveObj<bool>> lock(run_flag_);
                if(!run_flag_.obj) break;
            }

            if(x11_is_valid_) {
                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<StateEstimator>>> lock(state_estimator_);
                    particles  = state_estimator_.obj->getEstimateStates();
                    trajectory = state_estimator_.obj->getTrajectory();
                }

                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<ConstraintGenerator>>> lock(constraint_generator_);
                    route_info = constraint_generator_.obj->getRouteInfo();
                    tl_detection_result = constraint_generator_.obj->getTLDetectionResult();
                    pedestrian = constraint_generator_.obj->getPedestrianDetectionResult();
                }

                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<PathPlanner>>> lock(path_planner_);
                    path = path_planner_.obj->getResult();
                }

                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<PathTracer>>> lock(path_tracer_);
                    ref_state = path_tracer_.obj->getRefState();
                }

                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<PcamController>>> lock(pcam_ctrl_);
                    bin_img  = pcam_ctrl_.obj->getRoadSurfaceImage(PcamController::ImageType::BIN).clone();
                    edge_img = pcam_ctrl_.obj->getRoadSurfaceImage(PcamController::ImageType::EDGE).clone();
                    sat_img  = pcam_ctrl_.obj->getRoadSurfaceImage(PcamController::ImageType::SAT).clone();
                }

                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<WebcamController>>> lock(webcam_ctrl_);
                    webcam_img = webcam_ctrl_.obj->getWebCamImage().clone();
                }

                {
                    std::lock_guard<ExclusiveObj<std::unique_ptr<PatternMatchingVOCalculator>>> lock(vo_calc_);
                    vo = vo_calc_.obj->get(false);
                }

                const auto side_length = debug_param_.img_side_length;
                const auto debug_map = core::HandlerHelper::createCurrentWorld(side_length, world, route_info, ref_state, path, trajectory,
                                                                               particles, vo, tl_detection_result, pedestrian, webcam_img, bin_img, edge_img, sat_img);
                if(debug_param_.imwrite_mode) {
                    cv::imwrite("./img" + std::to_string(index) + ".png", debug_map);
                    if(index % debug_param_.interval_imshow_debug_map == 0) cv::imshow("Online Map", debug_map);
                    index++;
                }
                else {
                    cv::imshow("Online Map", debug_map);
                }
                const char key = cv::waitKey(1);
                if(key == 'q') {
                    std::lock_guard<ExclusiveObj<bool>> lock(run_flag_);
                    run_flag_.obj = false;
                }
            }

            timer.stop();
            {
                std::lock_guard<ExclusiveObj<AverageTime>> lock(average_time_);
                average_time_.obj.th_debagging = timer.getAverageElapsedTime();
                average_time = average_time_.obj;
            }

            std::cout << "----------- Elapsed Time -----------" << std::endl;
            std::cout << "WO Calculating  : " << std::setprecision(5) << average_time.th_wo_calculating * 1000 << "ms" << std::endl;
            std::cout << "IO Calculating  : " << std::setprecision(5) << average_time.th_io_calculating * 1000 << "ms" << std::endl;
            std::cout << "VO Calculating  : " << std::setprecision(5) << average_time.th_vo_calculating * 1000 << "ms" << std::endl;
            std::cout << "Localizationing : " << std::setprecision(5) << average_time.th_localizationing * 1000 << "ms" << std::endl;
            std::cout << "Planning        : " << std::setprecision(5) << average_time.th_planning * 1000 << "ms" << std::endl;
            std::cout << "Tracing         : " << std::setprecision(5) << average_time.th_tracing * 1000 << "ms" << std::endl;
            std::cout << "Debugging       : " << std::setprecision(5) << average_time.th_debagging * 1000 << "ms" << std::endl;
            std::cout << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds((uint32_t)(exec_period_.debagging * 1e6)));
        }
        cv::destroyAllWindows();
    }
}
