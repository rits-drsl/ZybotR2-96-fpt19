#include <linux/kernel.h>
#include <asm/io.h>
#include "xparameters.h"
#include "xcsi.h"
#include "zynq_v4l2.h"

#define MIPICSI_BASEADDR(num)  XPAR_CAMERA_INTERFACE_MIPI_CSI2_RX_SUBSYST_ ## num ## _BASEADDR
#define MIPICSI_HIGHADDR(num)  XPAR_CAMERA_INTERFACE_MIPI_CSI2_RX_SUBSYST_ ## num ## _HIGHADDR
#define MIPICSI_DEVICE_ID(num) XPAR_CAMERA_INTERFACE_MIPI_CSI2_RX_SUBSYST_ ## num ## _DEVICE_ID

int zynq_v4l2_mipicsi_init(void)
{
    int minor;
    XCsi ins;
    XCsi_Config *psConf;
    void __iomem *mem;
    XStatus Status;
    uint32_t baseaddr[MINOR_NUM] = {MIPICSI_BASEADDR(0)};
    uint32_t highaddr[MINOR_NUM] = {MIPICSI_HIGHADDR(0)};
    uint32_t device_id[MINOR_NUM] = {MIPICSI_DEVICE_ID(0)};

    printk(KERN_INFO "%s\n", __FUNCTION__);

    for (minor = 0; minor < MINOR_NUM; minor++) {
        mem = ioremap_nocache(baseaddr[minor], highaddr[minor] - baseaddr[minor]);
        if (!mem) {
            printk(KERN_ERR "ioremap_nocache failed %d\n", minor);
            return -ENOMEM;
        }

        psConf = XCsi_LookupConfig(device_id[minor]);
        if (!psConf) {
            printk(KERN_ERR "XCsiSs_LookupConfig failed %d\n", minor);
            iounmap(mem);
            return -ENODEV;
        }

        Status = XCsi_CfgInitialize(&ins, psConf, (UINTPTR)mem);
        if (Status != XST_SUCCESS){
            printk(KERN_ERR "XCsiSs_CfgInitialize failed %d\n", minor);
            iounmap(mem);
            return -ENODEV;
        }

        Status = XCsi_Reset(&ins);
        if (Status != XST_SUCCESS) {
            printk(KERN_ERR "XCsiSs_Reset failed %d\n", minor);
            iounmap(mem);
            return -ENODEV;
        }

        Status = XCsi_Activate(&ins, XCSI_ENABLE);
        if (Status != XST_SUCCESS) {
            printk(KERN_ERR "XCsiSs_Activate failed %d\n", minor);
            iounmap(mem);
            return -ENODEV;
        }
        iounmap(mem);
    }

    return 0;
}
