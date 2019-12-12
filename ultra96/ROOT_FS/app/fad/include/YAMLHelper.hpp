/**
 *  YAMLHelper: YAMLファイルの読み込みを簡単化するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_YAMLHELPER_HPP_
#define INCLUDE_YAMLHELPER_HPP_

#include <opencv2/core.hpp>
#include <iostream>
#include <string>

namespace core {
    class YAMLHelper {
    public:
        /**
         *  YAMLファイルから対象のラベル名のデータを読み込む
         *  存在しないラベルを指定した場合、std::runtime_errorを投げる
         *
         *  @filepath: YAMLファイルのパス
         *  @data:     読み出したデータを格納する変数の参照
         *  @labels:   対象のラベル名(階層構造になっている場合は複数渡すこと)
         *  @Return:   void
         */
        template<typename T, class... A>
        static void read(const std::string& filepath, T& data, const A&... labels) {
            cv::FileStorage fs(filepath, cv::FileStorage::Mode::READ);
            if(!fs.isOpened()) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] Can not open file : " + filepath);
            }

            cv::FileNode node;
            auto label_list = std::initializer_list<std::string>{labels...};
            auto cnt        = label_list.size();
            if(cnt != 0) {
                for(const auto& label : label_list) {
                    if(cnt == label_list.size()) {
                        node = fs[label];
                    }
                    else {
                        node = node[label];
                    }

                    cnt--;
                    if(cnt == 0) {
                        node >> data;
                    }
                }
            }

            if(node.empty()) {
                std::string labels = "";
                for(const auto& label : label_list) {
                    labels += label + " ";
                }
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                         "Label of data is invalid : " + labels);
            }
        }

        /**
         *  YAMLファイルから構造体を読み込むための基底クラス
         *  以下、使用例
         *  --------------------------------------------------------
         *  %YAML:1.0
         *  piyo:
         *     huga:
         *        A: 10
         *        B: "test"
         *  --------------------------------------------------------
         *  class ParamHoge : public core::YAMLHelper::ParamBase {
         *  public:
         *      uint32_t    A;
         *      std::string B;
         *      void read(const cv::FileNode& node) override {
         *          index  = (int)node["A"];
         *          number = (std::string)node["B"];
         *      }
         *  };
         *
         *  int main() {
         *      ParamHoge hoge;
         *      core::YAMLHelper::readStruct("test.yaml", hoge, "piyo", "huga");
         *      std::cout << "hoge.A : " << hoge.A << std::endl;  // 10
         *      std::cout << "hoge.B : " << hoge.B << std::endl;  // test
         *  }
         *  --------------------------------------------------------
         */
        class ParamBase {
        public:
            virtual void read(const cv::FileNode& node) = 0;
        };

        /**
         *  YAMLファイルから対象のラベル名のデータを構造体として読み込む
         *  存在しないラベルを指定した場合、std::runtime_errorを投げる
         *
         *  @filepath: YAMLファイルのパス
         *  @data:     読み出したデータを格納する構造体の参照
         *  @labels:   対象のラベル名(階層構造になっている場合は複数渡すこと)
         *  @Return:   void
         */
        template<class... A>
        static void readStruct(const std::string& filepath, ParamBase& data, const A&... labels) {
            cv::FileStorage fs(filepath, cv::FileStorage::Mode::READ);
            if(!fs.isOpened()) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] Can not open file: " + filepath);
            }

            cv::FileNode node;
            auto label_list = std::initializer_list<std::string>{labels...};
            auto cnt        = label_list.size();
            if(cnt != 0) {
                for(const auto& label : label_list) {
                    if(cnt == label_list.size()) {
                        node = fs[label];
                    }
                    else {
                        node = node[label];
                    }

                    cnt--;
                    if(cnt == 0) {
                        data.read(node);
                    }
                }
            }

            if(node.empty()) {
                std::string labels = "";
                for(const auto& label : label_list) {
                    labels += label + " ";
                }
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                         "Label of data is invalid : " + labels);
            }
        }
    };
}

#endif /* INCLUDE_YAMLHELPER_HPP_ */
