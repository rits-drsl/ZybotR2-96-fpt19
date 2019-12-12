#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include "zynq_v4l2.h"

extern int vdma_h_res, vdma_v_res;

long zynq_v4l2_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct v4l2_capability cap;
    struct v4l2_fmtdesc f;
    struct v4l2_frmsizeenum fsize;
    struct v4l2_frmivalenum fival;
    struct v4l2_format fmt;
    struct v4l2_streamparm streamparm;
    struct v4l2_queryctrl queryctrl;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    enum   v4l2_buf_type type;
    struct zynq_v4l2_dev_data *dp = filp->private_data;
    int rc, slot;
    #ifdef USE_VMALLOC
    unsigned long offset;
    #endif /* USE_VMALLOC */

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    switch (cmd) {
        case VIDIOC_QUERYCAP:
            cap.capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
            if (raw_copy_to_user((void __user *)arg, &cap, sizeof(cap))) {
                return -EFAULT;
            }
            return 0;
        case VIDIOC_ENUM_FMT:
            if (raw_copy_from_user(&f, (void __user *)arg, sizeof(f))) {
                return -EFAULT;
            }
            if (f.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            switch (f.index) {
                case 0:
                    f.pixelformat = V4L2_PIX_FMT_GREY;
                    strcpy(f.description, "GRAY");
                    rc = 0;
                    break;
                default:
                    rc = -EINVAL;
                    break;
            }
            if (rc == 0) {
                if (raw_copy_to_user((void __user *)arg, &f, sizeof(f))) {
                    return -EFAULT;
                }
            }
            return rc;
        case VIDIOC_ENUM_FRAMESIZES:
            if (raw_copy_from_user(&fsize, (void __user *)arg, sizeof(fsize))) {
                return -EFAULT;
            }
            switch (fsize.index) {
                case 0:
                    fsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
                    fsize.discrete.width = vdma_h_res;
                    fsize.discrete.height = vdma_v_res;
                    rc = 0;
                    break;
                default:
                    rc = -EINVAL;
            }
            if (rc == 0) {
                if (raw_copy_to_user((void __user *)arg, &fsize, sizeof(fsize))) {
                    return -EFAULT;
                }
            }
            return rc;
        case VIDIOC_ENUM_FRAMEINTERVALS:
            if (raw_copy_from_user(&fival, (void __user *)arg, sizeof(fival))) {
                return -EFAULT;
            }
            switch (fival.pixel_format) {
                case V4L2_PIX_FMT_GREY:
                    switch (fival.index) {
                        case 0:
                            if (fival.height != vdma_v_res) {
                                return -EINVAL;
                            }
                            if (fival.width != vdma_h_res) {
                                return -EINVAL;
                            }
                            fival.type = V4L2_FRMIVAL_TYPE_DISCRETE;
                            fival.discrete.numerator = 1;
                            fival.discrete.denominator = CAMERA_FPS;
                            rc = 0;
                            break;
                        default:
                            rc = -EINVAL;
                    }
                    break;
                default:
                    rc = -EINVAL;
            }
            if (rc == 0) {
                if (raw_copy_to_user((void __user *)arg, &fival, sizeof(fival))) {
                    return -EFAULT;
                }
            }
            return rc;
        case VIDIOC_G_FMT:
            if (raw_copy_from_user(&fmt, (void __user *)arg, sizeof(fmt))) {
                return -EFAULT;
            }
            switch (fmt.type) {
                case V4L2_BUF_TYPE_VIDEO_CAPTURE:
                    fmt.fmt.pix.width = dp->frame.width;
                    fmt.fmt.pix.height = dp->frame.height;
                    fmt.fmt.pix.pixelformat = dp->frame.pixelformat;
                    fmt.fmt.pix.field = V4L2_FIELD_ANY;
                    if (raw_copy_to_user((void __user *)arg, &fmt, sizeof(fmt))) {
                        return -EFAULT;
                    }
                    return 0;
                default:
                    return -ENOSYS;
            }
        case VIDIOC_S_FMT:
            if (raw_copy_from_user(&fmt, (void __user *)arg, sizeof(fmt))) {
                return -EFAULT;
            }
            if (fmt.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -ENOSYS;
            }
            if ((fmt.fmt.pix.height != vdma_v_res) || (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY)) {
                return -EINVAL;
            }
            if (fmt.fmt.pix.width != vdma_h_res) {
                return -EINVAL;
            }
            return 0;
        case VIDIOC_S_PARM:
            if (raw_copy_from_user(&streamparm, (void __user *)arg, sizeof(streamparm))) {
                return -EFAULT;
            }
            if (streamparm.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
                streamparm.parm.capture.timeperframe.numerator != 1 ||
                streamparm.parm.capture.timeperframe.denominator != CAMERA_FPS) {
                return -EINVAL;
            }
            return 0;
        case VIDIOC_QUERYCTRL:
            queryctrl.flags = V4L2_CTRL_FLAG_DISABLED;
            if (raw_copy_to_user((void __user *)arg, &queryctrl, sizeof(queryctrl))) {
                return -EFAULT;
            }
            return 0;
        case VIDIOC_REQBUFS:
            if (dp->mem.mmap) {
                return -EINVAL;
            }
            if (raw_copy_from_user(&req, (void __user *)arg, sizeof(req))) {
                return -EFAULT;
            }
            if (req.memory != V4L2_MEMORY_MMAP) {
                return -EINVAL;
            }
            if (req.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            if (req.count > MAX_USER_MEM) {
                return -EINVAL;
            }
            dp->frame.user_count = req.count;
            #ifdef USE_VMALLOC
            dp->mem.mmap = vmalloc(ALIGN(dp->frame.size, PAGE_SIZE) * req.count);
            #else /* !USE_VMALLOC */
            dp->mem.mmap = dma_alloc_coherent(dp->dma_dev, ALIGN(dp->frame.size, PAGE_SIZE) * req.count, &dp->mem.phys_mmap, GFP_KERNEL);
            #endif /* !USE_VMALLOC */
            PRINTK(KERN_INFO "dp->mem.mmap = %p\n", dp->mem.mmap);
            if (IS_ERR_OR_NULL(dp->mem.mmap)) {
                return -ENOMEM;
            }
            #ifdef USE_VMALLOC
            /* assign physical memory */
            offset = 0;
            while (offset < ALIGN(dp->frame.size, PAGE_SIZE) * req.count) {
                *((uint32_t *)((size_t)dp->mem.mmap + offset)) = 0;
                offset += PAGE_SIZE;
            }
            #endif /* USE_VMALLOC */
            return 0;
        case VIDIOC_QUERYBUF:
            if (raw_copy_from_user(&buf, (void __user *)arg, sizeof(buf))) {
                return -EFAULT;
            }
            if (buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            if (buf.memory != V4L2_MEMORY_MMAP) {
                return -EINVAL;
            }
            if (buf.index >= dp->frame.user_count) {
                return -EINVAL;
            }
            buf.length = dp->frame.size;
            buf.m.offset = ALIGN(dp->frame.size, PAGE_SIZE) * buf.index;
            if (raw_copy_to_user((void __user *)arg, &buf, sizeof(buf))) {
                return -EFAULT;
            }
            return 0;
        case VIDIOC_QBUF:
            if (raw_copy_from_user(&buf, (void __user *)arg, sizeof(buf))) {
                return -EFAULT;
            }
            if (buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            if (buf.memory != V4L2_MEMORY_MMAP && buf.memory != V4L2_MEMORY_USERPTR) {
                return -EINVAL;
            }
            if (buf.index >= dp->frame.user_count) {
                return -EINVAL;
            }
            spin_lock_irq(&dp->lock);
            dp->ctrl.queue_bits |= (1 << buf.index);
            PRINTK(KERN_INFO "QBUF: buf.index = %d, dp->ctrl.queue_bits = %d, dp->ctrl.active_bits = %d\n", buf.index, dp->ctrl.queue_bits, dp->ctrl.active_bits);
            if (buf.memory == V4L2_MEMORY_USERPTR) {
                dp->mem.userptr[buf.index] = buf.m.userptr;
            }
            spin_unlock_irq(&dp->lock);
            return 0;
        case VIDIOC_DQBUF:
            if (raw_copy_from_user(&buf, (void __user *)arg, sizeof(buf))) {
                return -EFAULT;
            }
            if (buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            if (buf.memory != V4L2_MEMORY_MMAP && buf.memory != V4L2_MEMORY_USERPTR) {
                return -EINVAL;
            }
            if (buf.index >= dp->frame.user_count) {
                return -EINVAL;
            }
            spin_lock_irq(&dp->lock);
            slot = zynq_v4l2_find_oldest_slot(dp->ctrl.active_bits, dp->ctrl.latest_frame);
            if (slot == -1 ) {
                spin_unlock_irq(&dp->lock);
                rc = -EAGAIN;
            } else {
                dp->ctrl.queue_bits &= ~(1 << slot);
                dp->ctrl.active_bits &= ~(1 << slot);
                PRINTK(KERN_INFO "DQBUF: slot = %d, dp->ctrl.queue_bits = %d, dp->ctrl.active_bits = %d\n", slot, dp->ctrl.queue_bits, dp->ctrl.active_bits);
                buf.index = slot;
                spin_unlock_irq(&dp->lock);
                buf.bytesused = dp->frame.size;
                if (raw_copy_to_user((void __user *)arg, &buf, sizeof(buf))) {
                    rc = -EFAULT;
                } else {
                    if (buf.memory == V4L2_MEMORY_USERPTR) {
                        int wb = zynq_v4l2_get_latest_frame_number(dp);
                        dma_sync_single_for_cpu(dp->dma_dev, dp->vdma.phys_wb + dp->frame.size * wb, dp->frame.size, DMA_FROM_DEVICE);
                        if (raw_copy_to_user((void __user *)dp->mem.userptr[slot],
                                             (void *)((unsigned long)dp->vdma.virt_wb + dp->frame.size * wb),
                                             dp->frame.size)) {
                            rc = -EFAULT;
                        } else {
                            rc = 0;
                        }
                    } else  {
                        rc = 0;
                    }
                }
                if (rc != 0) {
                    spin_lock_irq(&dp->lock);
                    dp->ctrl.queue_bits |= (1 << slot);
                    dp->ctrl.active_bits |= (1 << slot);
                    spin_unlock_irq(&dp->lock);
                }
            }
            return rc;
        case VIDIOC_STREAMON:
            if (raw_copy_from_user(&type, (void __user *)arg, sizeof(type))) {
                return -EFAULT;
            }
            if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            zynq_v4l2_vdma_intr_enable(dp);
            return 0;
        case VIDIOC_STREAMOFF:
            if (raw_copy_from_user(&type, (void __user *)arg, sizeof(type))) {
                return -EFAULT;
            }
            if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                return -EINVAL;
            }
            zynq_v4l2_vdma_intr_disable(dp);
            return 0;
        default:
            return -ENOSYS;
    }
}
