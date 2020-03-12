# ZybotR2-96-fpt19
An UGV-system using SoC-FPGA developed for FPGA design competition held on ICFPT2019.

**Our team won first place! :tada: :tada: :tada:**
<p align="center">
    <img src="https://github.com/rits-drsl/ZybotR2-96-fpt19/blob/media/ZybotR2-96.jpg" alt="ZybotR2-96" width="640px">
</p>

## Features
- **Localization** and **Motion Planning** are implemented in the UGV-system
    - In the **Localization** process, observations such as **Wheel Odometry** are fusioned by **Particle Filter**
    - In the **Motion Planning** process, **Informed-RRT\***, **Pure Pursuit**, and **PID control** are used
![debug](https://github.com/rits-drsl/ZybotR2-96-fpt19/blob/media/debug.gif)

- Efficient data flow that image data captured by image sensor are processed by FPGA before transferring to DRAM
- Peripheral devices (e.g. DC motor) can be handled transparently from Zynq PS by a system that synchronize registers between multiple FPGAs with UART
<p align="center">
    <img src="https://github.com/rits-drsl/ZybotR2-96-fpt19/blob/media/architecture.jpg" alt="architecture" width="640px">
</p>

- The UGV-system is divided into **five layers**, and it is descending order of abstraction
<p align="center">
    <img src="https://github.com/rits-drsl/ZybotR2-96-fpt19/blob/media/system_layer.jpg" alt="system_layer" width="320px">
</p>

## Main Parts of The Vehicle
- [Avnet Ultra96 V1](https://www.xilinx.com/products/boards-and-kits/1-vad4rl.html)
- [Digilent Cmod A7-35T](https://reference.digilentinc.com/reference/programmable-logic/cmod-a7/start/)
- [96Boards MIPI Adapter Mezzanine](https://www.96boards.org/product/mipiadapter/)
- [Pcam5C](https://store.digilentinc.com/pcam-5c-5-mp-fixed-focus-color-camera-module/)
- [Web Camera (Buffalo BSW20KM11BK)](https://www.buffalo.jp/product/detail/bsw20km11bk.html)
- [Stereo Camera (Optor)](https://www.seeedstudio.com/Optor-Cam2pc-Visual-Inertial-SLAM-p-307.html)
- [Anker PowerCore+ 26800 PD](https://www.anker.com/products/variant/powercore--26800-pd-with-30w-power-delivery-charger/B1375112)
- [DC Motor/Gearbox (1:53 Gear Ratio)](https://store.digilentinc.com/dc-motor-gearbox-1-53-gear-ratio-custom-6v-motor-designed-for-digilent-robot-kits/)

## Repository Structure
A part of this repository structure is shown below:

``` sh
ZybotR2-96-fpt19
├── assets
│   ├── cad
│   ├── script
│   └── tools
├── cmod-a7
│   └── vivado
│       ├── prj
│       └── src
└── ultra96
    ├── BOOT_FS
    ├── ROOT_FS
    │   ├── app
    │   │   ├── fad
    │   │   └── other
    │   ├── driver
    │   │   ├── cp210x
    │   │   ├── usbserial
    │   │   └── v4l2
    │   ├── dts
    │   ├── firmware
    │   ├── lib
    │   │   ├── control
    │   │   ├── improc
    │   │   ├── optor
    │   │   ├── planner
    │   │   └── zynqpl
    │   └── package
    └── vivado
        ├── ip
        ├── prj
        └── src
```

- `assets/`
    - 3DCAD data for the vehicle parts, scripts which install libraries such as OpenCV, and tools are stored
- `cmod/vivado/`
    - RTL files and scripts which create project are stored
    - A Registers synchronization module, a motor controller, and an OLED controller are implemented
- `ultra96/BOOT_FS/`
    - Binary files for booting Linux on Ultra96 are stored
    - These files are prepared with reference to [ikwzm/ZynqMP-FPGA-Linux](https://github.com/ikwzm/ZynqMP-FPGA-Linux)
- `ultra96/ROOT_FS/app/`
    - An application of FAD(FPGA Autonomous Driving), test applications of devices, and so on are stored
- `ultra96/ROOT_FS/lib/`
    - Shared libraries are stored
    - `planner` is based on [kyk0910/sampling-based-planners](https://github.com/kyk0910/sampling-based-planners)
    - `optor` is prepared with reference to [optor-vis/optor_vi-stereo-v1](https://github.com/optor-vis/optor_vi-stereo-v1)
- `ultra96/ROOT_FS/driver/`
    - Device drivers are stored
    - `v4l2` which is Linux V4L2 driver which deals with the Xilinx VDMA IP is prepared with reference to [fixstars/ultra96_design](https://github.com/fixstars/ultra96_design)
- `ultra96/ROOT_FS/dts/`
    - DTS(Device Tree Source) files are stored
    - We reflect some of device information to kernel using **Device Tree Overlay** which is feature of Linux kernel
- `ultra96/ROOT_FS/firmware/`
    - A bitstream file and firmware of wireless LAN modules are stored
    - A bitstream file is loaded with **FPGA Region** which is feature of Linux kernel
- `ultra96/ROOT_FS/package/`
    - Debian packages of **linux-headers** and **linux-image** are stored
    - There are also prepared with reference to [ikwzm/ZynqMP-FPGA-Linux](https://github.com/ikwzm/ZynqMP-FPGA-Linux)
- `ultra96/vivado/`
    - RTL files, HLS sources, IPs made by Digilent and scripts which create project are stored
    - `preimproc` module is based on [kyk0910/HLS-canny-edge-detection](https://github.com/kyk0910/HLS-canny-edge-detection)

## Future Work
- Offload computation heavy processes such as Informed-RRT*, Particle Filter, path tracking algorithms, and calculating Visual Odometry
- Implement more accurate image recognition algorithms

## Links
- [2019 International Conference on Field-Programmable Technology FPGA Design Competition](http://fpt19.tju.edu.cn/Contest/FPT2019_FPGA_Design_Competition.htm)

## Authors
- [Yuya Kudo](https://github.com/kyk0910)
- [Atsushi Takada](https://github.com/atlchemt)
- [Yuta Ishida](https://github.com/yukkuriDo)
- Prof. Tomonori Izumi

## References
[Y. Kudo, A. Takada, Y. Ishida, and T. Izumi, "An SoC-FPGA-based Micro UGV with Localization and Motion Planning," in Proceedings of 2019 International Conference on Field-Programmable Technology (ICFPT), 2019, pp. 469-472.](https://ieeexplore.ieee.org/document/8977887)

## License
MIT
