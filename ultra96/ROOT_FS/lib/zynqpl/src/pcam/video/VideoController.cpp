/**
 *  VideoController: V4L2を用いた画像取得を簡単化するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <pcam/video/VideoController.h>

namespace zynqpl {
    VideoController::VideoController(const std::string& video_devname,
                                     u_int32_t width,
                                     u_int32_t height,
                                     u_int32_t pixelformat) :
        selected_buf_index_(-1)
    {
        this->v4l2_fd_ = open(("/dev/" + video_devname).c_str(), O_RDWR);
        if(this->v4l2_fd_ < 0) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not open video device : " + std::string(strerror(errno)));
        }

        v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));

        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = width;
        fmt.fmt.pix.height      = height;
        fmt.fmt.pix.pixelformat = pixelformat;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        this->xioctl(VIDIOC_S_FMT, &fmt, "VIDIOC_S_FMT");

        v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));

        req.count  = NUM_BUFFER;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        this->xioctl(VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS");

        v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.memory = V4L2_MEMORY_MMAP;
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for(int i = 0; i < NUM_BUFFER; i++) {
            buf.index = i;
            this->xioctl(VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF");

            this->v4l2_user_frame_[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, this->v4l2_fd_, buf.m.offset);
            if(!this->v4l2_user_frame_[i] || this->v4l2_user_frame_[i] == (void*) - 1) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                         "mapping of user frame is failure : " + std::string(strerror(errno)));
            }

            buf_size = buf.length;

            this->xioctl(VIDIOC_QBUF, &buf, "VIDIOC_QBUF");
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->xioctl(VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    }

    VideoController::~VideoController() {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        this->xioctl(VIDIOC_STREAMOFF, &type, "VIDIOC_STREAMOFF");
        close(this->v4l2_fd_);
    }

    const uint8_t* VideoController::grub() {
        v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.memory = V4L2_MEMORY_MMAP;
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(this->v4l2_fd_, &fds);

        timeval tv;
        tv.tv_sec  = 2;
        tv.tv_usec = 0;
        select(this->v4l2_fd_ + 1, &fds, NULL, NULL, &tv);

        if(FD_ISSET(this->v4l2_fd_, &fds)) {
            this->xioctl(VIDIOC_DQBUF, &buf, "VIDIOC_DQBUF");
            this->selected_buf_index_ = buf.index;

            if(NUM_BUFFER <= this->selected_buf_index_) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                         "selected buffer index is invalid : " + std::string(strerror(errno)));
            }
        }
        else {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "select() failure : " + std::string(strerror(errno)));
        }

        return (uint8_t*)v4l2_user_frame_[this->selected_buf_index_];
    }

    void VideoController::release() {
        v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.memory = V4L2_MEMORY_MMAP;
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.index  = this->selected_buf_index_;

        this->xioctl(VIDIOC_QBUF, &buf, "VIDIOC_QBUF");
    }

    void VideoController::xioctl(int request, void *arg, const std::string& req_description) {
        int rc;
        do {
            rc = ioctl(this->v4l2_fd_, request, arg);
        }
        while(-1 == rc && EINTR == errno);

        if(rc < 0) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "ioctl " + req_description + " : " + std::string(strerror(errno)));
        }
    }
}
