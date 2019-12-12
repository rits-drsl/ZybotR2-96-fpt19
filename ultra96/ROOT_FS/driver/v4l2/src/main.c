#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/videodev2.h>
#include "zynq_v4l2.h"

MODULE_DESCRIPTION("ZYNQ v4l2 device driver");
MODULE_AUTHOR("takashi.osawa");
MODULE_LICENSE("GPL");

int vdma_h_res = 1280;
int vdma_v_res = 720;
module_param(vdma_h_res, int, S_IRUGO);
module_param(vdma_v_res, int, S_IRUGO);
MODULE_PARM_DESC(vdma_h_res, "v4l2 buffer width");
MODULE_PARM_DESC(vdma_v_res, "v4l2 buffer height");

int zynq_v4l2_find_oldest_slot(uint32_t slot_bits, int latest)
{
    uint32_t bits = slot_bits;

    bits &= ~(((1 << latest) << 1) - 1);
    if (bits ) {
        return __builtin_ctz(bits);
    } else if (slot_bits) {
        return __builtin_ctz(slot_bits);
    } else {
        return -1;
    }
}

static int zynq_v4l2_open(struct inode *inode, struct file *filp)
{
    struct zynq_v4l2_sys_data *sp;
    struct zynq_v4l2_dev_data *dp;
    int minor;

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    sp = container_of(inode->i_cdev, struct zynq_v4l2_sys_data, cdev);
    if (sp == NULL) {
        printk(KERN_ERR "container_of failed\n");
        return -EFAULT;
    }
    minor = iminor(inode);
    dp = &sp->dev[minor];
    filp->private_data = dp;

    return 0;
}

static int zynq_v4l2_close(struct inode *inode, struct file *filp)
{
    struct zynq_v4l2_dev_data *dp = filp->private_data;

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    zynq_v4l2_vdma_intr_disable(dp);
    spin_lock_irq(&dp->lock);
    dp->ctrl.queue_bits = 0;
    dp->ctrl.active_bits = 0;
    spin_unlock_irq(&dp->lock);
    while (dp->ctrl.work_queue_active) {
        schedule();
    }
    if (dp->mem.mmap) {
        #ifdef USE_VMALLOC
        vfree(dp->mem.mmap);
        #else /* !USE_VMALLOC */
        dma_free_coherent(dp->dma_dev, ALIGN(dp->frame.size, PAGE_SIZE) * dp->frame.user_count, dp->mem.mmap, dp->mem.phys_mmap);
        #endif /* !USE_VMALLOC */
        dp->mem.mmap = NULL;
    }

    return 0;
}

static ssize_t zynq_v4l2_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct zynq_v4l2_dev_data *dp = filp->private_data;
    int wb;

    if (count > vdma_h_res * vdma_v_res * 3) {
        return -EINVAL;
    }

    wb = XAxiVdma_CurrFrameStore(dp->vdma.inst, XAXIVDMA_WRITE);
    wb = (wb + dp->vdma.inst->MaxNumFrames - 1) % dp->vdma.inst->MaxNumFrames;
    printk(KERN_INFO "%s wb = %d\n", __FUNCTION__, wb);
    if (raw_copy_to_user(buf, dp->vdma.virt_wb + dp->frame.size * wb, count) != 0) {
        return -EFAULT;
    }

    return count;
}

static ssize_t zynq_v4l2_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    return 0;
}

static unsigned int zynq_v4l2_poll(struct file *filp, struct poll_table_struct *waitp)
{
    struct zynq_v4l2_dev_data *dp = filp->private_data;
    int slot, rc;

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    spin_lock_irq(&dp->lock);
    slot = zynq_v4l2_find_oldest_slot(dp->ctrl.active_bits, dp->ctrl.latest_frame);
    spin_unlock_irq(&dp->lock);

    if (slot == -1) {
        poll_wait(filp, &dp->ctrl.waitq, waitp);
        spin_lock_irq(&dp->lock);
        if ((slot = zynq_v4l2_find_oldest_slot(dp->ctrl.active_bits, dp->ctrl.latest_frame)) == -1) {
            rc = 0;
        } else {
            rc = (POLLIN | POLLRDNORM);
        }
        spin_unlock_irq(&dp->lock);
    } else {
        rc = (POLLIN | POLLRDNORM);
    }
    PRINTK(KERN_INFO "  rc = %d, slot = %d\n", rc, slot);

    return rc;
}

static int zynq_v4l2_vma_fault(struct vm_fault *vmf)
{
    struct page *page;
    unsigned long offset = vmf->pgoff << PAGE_SHIFT;
    struct zynq_v4l2_dev_data *dp = vmf->vma->vm_private_data;

    if (offset > dp->frame.size * dp->frame.user_count) {
        return VM_FAULT_SIGBUS;
    }

    #ifdef USE_VMALLOC
    page = vmalloc_to_page((void *)((unsigned long)dp->mem.mmap + offset));
    #else /* !USE_VMALLOC */
    page = phys_to_page(dp->mem.phys_mmap + offset);
    #endif /* !USE_VMALLOC */
    get_page(page);
    vmf->page = page;

    return 0;
}

static struct vm_operations_struct zynq_v4l2_vm_ops = {
                                                       .fault = zynq_v4l2_vma_fault,
};

static int zynq_v4l2_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct zynq_v4l2_dev_data *dp = filp->private_data;

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    if ((vma->vm_pgoff + vma_pages(vma)) > ((ALIGN(dp->frame.size, PAGE_SIZE) * dp->frame.user_count) >> PAGE_SHIFT)) {
        return -EINVAL;
    }

    vma->vm_private_data = dp;
    vma->vm_ops = &zynq_v4l2_vm_ops;

    return 0;
}

static struct file_operations zynq_v4l2_fops = {
                                                .open           = zynq_v4l2_open,
                                                .release        = zynq_v4l2_close,
                                                .read           = zynq_v4l2_read,
                                                .write          = zynq_v4l2_write,
                                                .poll           = zynq_v4l2_poll,
                                                .unlocked_ioctl = zynq_v4l2_ioctl,
                                                .compat_ioctl   = zynq_v4l2_ioctl,
                                                .mmap           = zynq_v4l2_mmap,
};

static int zynq_v4l2_create_cdev(struct device *parent, struct zynq_v4l2_sys_data *sp)
{
    int ret;
    dev_t dev;
    int minor;

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    ret = alloc_chrdev_region(&dev, MINOR_BASE, MINOR_NUM, DRIVER_NAME);
    if (ret != 0) {
        printk(KERN_ERR "alloc_chrdev_region = %d\n", ret);
        return -1;
    }

    sp->class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(sp->class)) {
        printk(KERN_ERR "class_create\n");
        goto error;
    }

    cdev_init(&sp->cdev, &zynq_v4l2_fops);
    sp->cdev.owner = THIS_MODULE;
    sp->major = MAJOR(dev);

    ret = cdev_add(&sp->cdev, dev, MINOR_NUM);
    if (ret != 0) {
        printk(KERN_ERR "cdev_add = %d\n", ret);
        goto error;
    }
    sp->cdev_inited = true;

    for (minor = 0; minor < MINOR_NUM; minor++) {
        struct zynq_v4l2_dev_data *dp = &sp->dev[minor];

        device_create(sp->class, parent, MKDEV(sp->major, minor), dp, "pcam%d", minor);
        dp->dma_dev = parent;
    }

    return 0;

 error:
    for (minor = 0; minor < MINOR_NUM; minor++) {
        if (sp->class) {
            device_destroy(sp->class, MKDEV(sp->major, minor));
        }
    }
    if (sp->cdev_inited) {
        cdev_del(&sp->cdev);
        sp->cdev_inited = false;
    }
    if (sp->class) {
        class_destroy(sp->class);
        sp->class = NULL;
    }
    unregister_chrdev_region(dev, MINOR_NUM);

    return -1;
}

static void zynq_v4l2_delete_cdev(struct zynq_v4l2_sys_data *sp)
{
    int minor;
    dev_t dev = MKDEV(sp->major, MINOR_BASE);

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    for (minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) {
        device_destroy(sp->class, MKDEV(sp->major, minor));
    }
    if (sp->cdev_inited) {
        cdev_del(&sp->cdev);
    }
    class_destroy(sp->class);
    unregister_chrdev_region(dev, MINOR_NUM);
}

static int zynq_v4l2_hw_init(struct device *dev, struct zynq_v4l2_sys_data *sp)
{
    int minor, rc;

    PRINTK(KERN_INFO "%s\n", __FUNCTION__);

    if (zynq_v4l2_vdma_init(dev, sp)) {
        return -ENODEV;
    }

    /* if (zynq_v4l2_demosaic_init()) { */
    /*      rc = -ENODEV; */
    /*      goto error; */
    /* } */

    if (zynq_v4l2_mipicsi_init()) {
        rc = -ENODEV;
        goto error;
    }

    return 0;

 error:
    for (minor = 0; minor < MINOR_NUM; minor++) {
        struct zynq_v4l2_dev_data *dp = &sp->dev[minor];

        free_irq(dp->vdma.irq_num, dp);
        dma_free_coherent(dp->dma_dev, dp->vdma.alloc_size_wb, dp->vdma.virt_wb, dp->vdma.phys_wb);
        iounmap(dp->vdma.reg);
    }

    return rc;
}

static void zynq_v4l2_free_data_all(struct zynq_v4l2_sys_data *sp)
{
    int minor;

    for (minor = 0; minor < MINOR_NUM; minor++) {
        struct zynq_v4l2_dev_data *dp = &sp->dev[minor];

        if (dp->vdma.irq_num) {
            free_irq(dp->vdma.irq_num, dp);
        }
        if (dp->vdma.virt_wb) {
            dma_free_coherent(dp->dma_dev, dp->vdma.alloc_size_wb, dp->vdma.virt_wb, dp->vdma.phys_wb);
        }
        if (dp->vdma.reg) {
            iounmap(dp->vdma.reg);
        }
        if (dp->mem.mmap) {
            #ifdef USE_VMALLOC
            vfree(dp->mem.mmap);
            #else /* !USE_VMALLOC */
            dma_free_coherent(dp->dma_dev, ALIGN(dp->frame.size, PAGE_SIZE) * dp->frame.user_count, dp->mem.mmap, dp->mem.phys_mmap);
            #endif /* !USE_VMALLOC */
        }
    }
    if (sp->cdev_inited) {
        zynq_v4l2_delete_cdev(sp);
    }
    if (sp->wq) {
        flush_workqueue(sp->wq);
        destroy_workqueue(sp->wq);
    }
    kfree(sp);
}

static int zynq_v4l2_probe(struct platform_device *pdev)
{
    int minor;
    struct device *dev = &pdev->dev;
    struct zynq_v4l2_sys_data *sp;

    dev_info(dev, "Device Tree Probing");
    printk(KERN_INFO "%s\n", __FUNCTION__);

    sp = (struct zynq_v4l2_sys_data *)kzalloc(sizeof(struct zynq_v4l2_sys_data), GFP_KERNEL);
    if (!sp) {
        dev_err(dev, "Could not allocate memory\n");
        return -ENOMEM;
    }

    if (zynq_v4l2_create_cdev(&pdev->dev, sp)) {
        dev_err(dev, "zynq_v4l2_create_cdev failed\n");
        zynq_v4l2_free_data_all(sp);
        dev_set_drvdata(dev, NULL);
        return -EBUSY;
    }

    if (zynq_v4l2_hw_init(&pdev->dev, sp)) {
        dev_err(dev, "hw_init failed\n");
        zynq_v4l2_free_data_all(sp);
        dev_set_drvdata(dev, NULL);
        return -ENODEV;
    }

    sp->wq = create_workqueue("zynq_v4l2_wq");
    if (!sp->wq) {
        dev_err(dev, "create_workqueue failed\n");
        zynq_v4l2_free_data_all(sp);
        dev_set_drvdata(dev, NULL);
        return -ENODEV;
    }

    for (minor = 0; minor < MINOR_NUM; minor++) {
        struct zynq_v4l2_dev_data *dp = &sp->dev[minor];

        spin_lock_init(&dp->lock);
        init_waitqueue_head(&dp->ctrl.waitq);
        dp->ctrl.latest_frame = dp->vdma.inst->MaxNumFrames - 1;
        dp->frame.user_count = MAX_USER_MEM;
        dp->sp = sp;
    }

    dev_set_drvdata(dev, sp);

    return 0;
}

static int zynq_v4l2_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct zynq_v4l2_sys_data *sp = dev_get_drvdata(dev);

    printk(KERN_INFO "%s\n", __FUNCTION__);

    zynq_v4l2_free_data_all(sp);
    dev_set_drvdata(dev, NULL);

    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id zynq_v4l2_of_match[] = {
                                                   { .compatible = "fixstars,zynq-v4l2-1.0", },
                                                   { /* end of table */ },
};
MODULE_DEVICE_TABLE(of, zynq_v4l2_of_match);
#else
#define zynq_v4l2_of_match
#endif

static struct platform_driver zynq_v4l2_driver = {
                                                  .driver = {
                                                             .name = DRIVER_NAME,
                                                             .owner = THIS_MODULE,
                                                             .of_match_table = zynq_v4l2_of_match,
                                                             },
                                                  .probe = zynq_v4l2_probe,
                                                  .remove = zynq_v4l2_remove,
};

static int zynq_v4l2_init(void)
{
    printk(KERN_INFO "%s\n", __FUNCTION__);

    return platform_driver_register(&zynq_v4l2_driver);
}

static void zynq_v4l2_exit(void)
{
    printk(KERN_INFO "%s\n", __FUNCTION__);
    platform_driver_unregister(&zynq_v4l2_driver);
}

module_init(zynq_v4l2_init);
module_exit(zynq_v4l2_exit);
