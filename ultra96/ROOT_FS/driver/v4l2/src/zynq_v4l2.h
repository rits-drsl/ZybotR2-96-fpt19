#ifndef ZYNQ_V4L2_H
#define ZYNQ_V4L2_H

#include <linux/cdev.h>
#include <linux/videodev2.h>
#include "xaxivdma.h"

#ifdef DEBUG
#define PRINTK(fmt, ...) printk(fmt, __VA_ARGS__)
#else
#define PRINTK(fmt, ...)
#endif

#define USE_VMALLOC

#define DRIVER_NAME  "v4l2"
#define MINOR_BASE   0
#define MINOR_NUM    1
#define MAX_USER_MEM 32
#define CAMERA_FPS   30

struct zynq_v4l2_sys_data;

struct zynq_v4l2_frm_data {
    size_t             size;
    size_t             width;
    size_t             height;
    __u32              pixelformat;
    int                user_count;
    struct v4l2_buffer buf[MAX_USER_MEM];
};

struct zynq_v4l2_ctrl_data {
    uint32_t          queue_bits;
    uint32_t          active_bits;
    int               latest_frame;
    wait_queue_head_t waitq;
    volatile bool     work_queue_active;
};

struct zynq_v4l2_mem_data {
    void          *mmap;
    #ifndef USE_VMALLOC
    dma_addr_t     phys_mmap;
    #endif /* USE_VMALLOC */
    unsigned long  userptr[MAX_USER_MEM];
};

struct zynq_v4l2_vdma_data {
    XAxiVdma     *inst;
    void __iomem *reg;
    dma_addr_t    phys_wb;
    void         *virt_wb;
    size_t        alloc_size_wb;
    int           irq_num;
};

struct zynq_v4l2_dev_data {
    struct device              *dma_dev;
    struct zynq_v4l2_sys_data  *sp;
    struct zynq_v4l2_frm_data   frame;
    struct zynq_v4l2_ctrl_data  ctrl;
    struct zynq_v4l2_mem_data   mem;
    struct zynq_v4l2_vdma_data  vdma;
    spinlock_t                  lock;
};

struct zynq_v4l2_sys_data {
    unsigned int               major;
    struct class              *class;
    struct cdev                cdev;
    bool                       cdev_inited;
    struct workqueue_struct   *wq;
    struct zynq_v4l2_dev_data  dev[MINOR_NUM];
};

struct zynq_v4l2_work_struct {
    struct work_struct         work;
    struct zynq_v4l2_dev_data *dp;
};

long zynq_v4l2_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int  zynq_v4l2_vdma_init(struct device *dev, struct zynq_v4l2_sys_data *sp);
void zynq_v4l2_vdma_intr_enable(struct zynq_v4l2_dev_data *dp);
void zynq_v4l2_vdma_intr_disable(struct zynq_v4l2_dev_data *dp);
int  zynq_v4l2_demosaic_init(void);
int  zynq_v4l2_mipicsi_init(void);
int  zynq_v4l2_find_oldest_slot(uint32_t active_bits, int latest);
int  zynq_v4l2_get_latest_frame_number(struct zynq_v4l2_dev_data *dp);

#endif /* ZYNQ_V4L2_H */
