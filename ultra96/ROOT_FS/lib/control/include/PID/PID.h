/**
 *  PID: PID制御を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 *
 */

#ifndef CONTROL_INCLUDE_PID_PID_H_
#define CONTROL_INCLUDE_PID_PID_H_

namespace control {
    class PID {
    public:
        struct Gain {
            double P;
            double I;
            double D;
        };

        explicit PID(const Gain& gain,
                     const double& target_val = 0);

        PID(const double& P_gain,
            const double& I_gain,
            const double& D_gain,
            const double& target_val = 0);

        ~PID();

        void setTargetValue(const double& val);

        void setGain(const Gain& gain);

        void setGain(const double& P_gain,
                     const double& I_gain,
                     const double& D_gain);

        double getOutValue() const;

        void init();

        void updete(const double& val);

    private:
        Gain   gain_;
        double target_val_;
        double out_val_;
        double prev_error_;
        double i_error_;
        bool   prev_error_is_valid_;
    };
}

#endif /* CONTROL_INCLUDE_PID_PID_H_ */
