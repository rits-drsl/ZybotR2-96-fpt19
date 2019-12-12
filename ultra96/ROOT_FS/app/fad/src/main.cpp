/**
 *  fpga-autonomous-driving: FPGAを用いた自動走行コンテスト向けのプログラム
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "main.h"

int main() {
    try {
        fad::Handler handler;
        auto exit_flag = false;
        while(!exit_flag) {
            auto selected_mode = '0';
            std::cout << std::endl << " ---------------- FPGA Design Competition Top Menu ---------------- " << std::endl;
            std::cout << "1. Launch Normal Mode" << std::endl;
            std::cout << "2. Launch Debug Mode" << std::endl;
            std::cout << "q. Quit" << std::endl << std::endl;
            std::cout << "Please input menu number : ";
            std::cin  >> selected_mode;
            switch(selected_mode) {
                case '1': {
                    std::cout << std::endl << " ---------------- Launch Normal Mode ---------------- " << std::endl;
                    handler.run(fad::Handler::Mode::NORMAL);
                    break;
                }
                case '2': {
                    std::cout << std::endl << " ---------------- Launch Debug Mode ---------------- " << std::endl;
                    handler.run(fad::Handler::Mode::DEBUG);
                    break;
                }
                case 'q': {
                    exit_flag = true;
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
    catch(const std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    catch(...) {
        std::cout << "An unexpected exception has occurred" << std::endl;
    }

    return 0;
}
