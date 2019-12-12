/**
 *  CSVDumper: 任意のデータをcsvファイルに書き込むクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_CSVDUMPER_HPP_
#define INCLUDE_CSVDUMPER_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

namespace core {
    class CSVDumper {
    public:
        template<class... A>
        CSVDumper(const A&... col_names) : max_data_length_(0) {
            auto list = std::initializer_list<std::string>{col_names...};
            for(const auto& v : list) {
                data_.emplace(v, std::vector<std::string>());
            }
        }
        ~CSVDumper() { }

        void push(const std::string& col_name, const std::string& data) {
            auto& v = data_[col_name];
            v.push_back(data);
            max_data_length_ = std::max(max_data_length_, v.size());
        }

        void write(const std::string& path) {
            std::ofstream ofs;
            if(ofs) {
                ofs.open(path, std::ios::out);
                // カラム名を書き込む
                for(const auto& v : data_) {
                    ofs << v.first << ",";
                }
                ofs << std::endl;

                // データを書き込む
                for(size_t i = 0; i < max_data_length_; i++) {
                    for(const auto& v : data_) {
                        if(i < v.second.size()) {
                            ofs << v.second[i];
                        }
                        ofs << ",";
                    }
                    ofs << std::endl;
                }
            }
            else {
                std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Can't open file");
            }
        }

    private:
        std::map<std::string, std::vector<std::string>> data_;
        size_t max_data_length_;
    };
}

#endif /* INCLUDE_CSVDUMPER_HPP_ */
