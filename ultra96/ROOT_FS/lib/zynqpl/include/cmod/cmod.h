/**
 *  Cmod: Cmod-A7に実装されているモジュールを制御するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef ZYNQPL_INCLUDE_CMOD_CMOD_H_
#define ZYNQPL_INCLUDE_CMOD_CMOD_H_

#include <iostream>
#include <memory>

#include <cstdio>
#include <cstdlib>
#include <climits>

#include <unistd.h>
#include <cmod/reg/RegAccessor.h>

namespace zynqpl {
    namespace motor {
        //--- レジスタのアドレスとデータの情報
        enum class ID { LEFT, RIGHT };
        enum class Gain { PROPORTION, INTEGRATION, DERIVATIVE };

        //-- Ultra96 -> CMod-A7
        // 目標角速度(8ビット左シフト)
        // key -> ID
        const uint8_t ADDR_ROTATION_VELOCITY[] = {0x00, 0x01};

        // PID制御のゲイン(8ビット左シフト)
        // key -> ID, Gain
        const uint8_t ADDR_PWM_GAIN[][2] = { {0x02, 0x03},
                                             {0x04, 0x05},
                                             {0x06, 0x07} };
        //-- CMod-A7 -> Ultra96
        // ロータリーエンコーダのカウンタ値
        // key -> ID
        const uint8_t ADDR_ROT_CNT[] = {0x80, 0x81};

        // Cmod-A7bに搭載されているボタンの状態
        const uint8_t  ADDR_CMOD_BTN  = 0x90;
        const uint32_t CMOD_BTN1_MASK = 0x00000001;
        const uint32_t CMOD_BTN2_MASK = 0x00000002;
    }

    /**
     *  Cmod
     *  Cmod-A7に実装されているモジュールを制御するクラス
     */
    class Cmod {
    public:
        /**
         *  Cmod(Constructor)
         *  @gpio1_devname: /dev/下にあるdataの入出力を行うAXI GPIOのデバイスファイル名
         *  @gpio2_devname: /dev/下にある制御信号の入出力を行うAXI GPIOのデバイスファイル名
         */
        Cmod(const std::string& gpio1_devname,
             const std::string& gpio2_devname);
        ~Cmod();

        Cmod(const Cmod& obj) = delete;
        Cmod &operator =(const Cmod& obj) = delete;

        /**
         *  モータに目標角速度を設定する
         *  @anguler_velocity: 設定する目標角速度
         *  @id: 目標角速度を設定するモータのID
         */
        void setAngulerVelocity(const double& anguler_velocity, const motor::ID& id) const;

        /**
         *  PWM波形出力回路のPIDゲインを設定する
         *  @gain: 設定するゲイン
         *  @id:   ゲインを設定するモータのID
         *  @type: ゲインの種類
         */
        void setPIDParameter(const double& gain, const motor::ID& id, const motor::Gain& type) const;

        /**
         *  ロータリーエンコーダのカウンタ値を取得する
         *  @id: カウンタ値を取得するモータのID
         */
        int getRotCount(const motor::ID& id) const;

        /**
         *  Cmodに搭載されているBTN1の状態を取得する
         */
        bool getCmodBTN1Status() const;

        /**
         *  Cmodに搭載されているBTN2の状態を取得する
         */
        bool getCmodBTN2Status() const;

        /**
         *  Cmodと同期しているレジスタをすべて初期化する
         */
        void initRegs() const;

    private:
        std::unique_ptr<RegAccessor> reg_accessor_;
    };
}

#endif /* ZYNQPL_INCLUDE_CMOD_CMOD_H_ */

