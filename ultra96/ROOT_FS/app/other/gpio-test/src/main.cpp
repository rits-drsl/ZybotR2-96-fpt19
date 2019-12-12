#include <iostream>
#include <string>
#include <utility>
#include <stdexcept>
#include <cstring>
#include <cstdint>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define REG(base, offset) *((volatile uint32_t*)((uintptr_t)(base) + (offset)))

#define XGPIO_DATA_OFFSET   0x0   /**< Data register for 1st channel */
#define XGPIO_DATA2_OFFSET  0x8   /**< Data register for 2nd channel */

int gpio0_fd, gpio1_fd;
void* gpio0_baseaddr;
void* gpio1_baseaddr;

int read_reg(uint8_t addr) {
    volatile int data;
    REG(gpio1_baseaddr, XGPIO_DATA_OFFSET) = (uint32_t)(0x00000100 + addr);
    while(!REG(gpio1_baseaddr, XGPIO_DATA2_OFFSET)) {
        usleep(10);
    }
    data = (int)REG(gpio0_baseaddr, XGPIO_DATA2_OFFSET);
    REG(gpio1_baseaddr, XGPIO_DATA_OFFSET) = (uint32_t)(0);

    return data;
}

void write_reg(uint32_t data, uint8_t addr) {
    REG(gpio0_baseaddr, XGPIO_DATA_OFFSET) = (uint32_t)(data);
    REG(gpio1_baseaddr, XGPIO_DATA_OFFSET) = (uint32_t)(0x00000300 + addr);
    while(!REG(gpio1_baseaddr, XGPIO_DATA2_OFFSET)) {
        usleep(10);
    }
    REG(gpio0_baseaddr, XGPIO_DATA_OFFSET) = (uint32_t)(0);
    REG(gpio1_baseaddr, XGPIO_DATA_OFFSET) = (uint32_t)(0);
}

int main() {
    try {
        // deviceのopen
        gpio0_fd = open("/dev/uio1", O_RDWR | O_SYNC);
        if(!gpio0_fd) {
            throw std::runtime_error("could not open device : " + std::string(strerror(errno)));
        }
        gpio1_fd = open("/dev/uio2", O_RDWR | O_SYNC);
        if(!gpio1_fd) {
            throw std::runtime_error("could not open device : " + std::string(strerror(errno)));
        }

        // baseaddrの取得
        gpio0_baseaddr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, gpio0_fd, 0);
        if(gpio0_baseaddr == MAP_FAILED) {
            throw std::runtime_error("could not get baseaddr : " + std::string(strerror(errno)));
        }
        gpio1_baseaddr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, gpio1_fd, 0);
        if(gpio1_baseaddr == MAP_FAILED) {
            throw std::runtime_error("could not get baseaddr : " + std::string(strerror(errno)));
        }
    }
    catch(const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
        return 0;
    }

    bool end_flag = false;
    while(!end_flag) {
        char number;
        std::cout << "Please input number" << std::endl;
        std::cout << "1. write" << std::endl;
        std::cout << "2. read" << std::endl;
        std::cout << "3. init" << std::endl;
        std::cout << "4. read once" << std::endl;
        std::cout << "q. exit" << std::endl;
        std::cin >> number;
        switch (number) {
            case '1': {
                uint32_t addr, data;
                std::cout << "addr : ";
                std::cin >> addr;
                std::cout << "data : ";
                std::cin >> data;

                write_reg(data, addr);
                break;
            }
            case '2': {
                for(int addr = 0; addr < 256; addr++) {
                    std::cout << std::hex << addr << " : " << read_reg(addr) << std::dec << std::endl;
                }
                break;
            }
            case '3': {
                for(int addr = 0; addr < 256; addr++) {
                    write_reg(0, addr);
                }

                break;
            }
            case '4': {
                std::string addr;
                std::cout << "addr : ";
                std::cin >> addr;
                std::cout << std::hex << addr << " : " << read_reg(std::stoi(addr)) << std::dec << std::endl;
                break;
            }
            case 'q': {
                end_flag = true;
                break;
            }
            default: break;
        }
    }

    close(gpio0_fd);
    close(gpio1_fd);
    munmap(gpio0_baseaddr, 0x1000);
    munmap(gpio1_baseaddr, 0x1000);
    return 0;
}
