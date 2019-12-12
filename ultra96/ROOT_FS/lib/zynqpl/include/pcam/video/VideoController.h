/**
 *  VideoController: V4L2を用いた画像取得を簡単化するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef ZYNQPL_INCLUDE_PCAM_VIDEO_VIDEOCONTROLLER_H_
#define ZYNQPL_INCLUDE_PCAM_VIDEO_VIDEOCONTROLLER_H_

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <linux/videodev2.h>

namespace zynqpl {
    class VideoController {
    public:
        uint32_t  buf_size;
        VideoController(const std::string& video_devname,
                        uint32_t width,
                        uint32_t height,
                        uint32_t pixelformat);
        ~VideoController();
        const uint8_t* grub();
        void release();

    private:
        const int NUM_BUFFER = 2;
        int       v4l2_fd_;
        void*     v4l2_user_frame_[2];
        int       selected_buf_index_;

        void xioctl(int request, void *arg, const std::string& req_description = "");
    };
}

#endif /* ZYNQPL_INCLUDE_PCAM_VIDEO_VIDEOCONTROLLER_H_ */
