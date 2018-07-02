/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   HC-SR04 Ultrasonic Distance Sensor connected to BeagleBone Black
   by Mirko Denecke <mirkix@gmail.com>
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI

#include "AP_RangeFinder_BBB_PRU.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>

extern const AP_HAL::HAL& hal;

volatile struct range *rangerpru;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_BBB_PRU::AP_RangeFinder_BBB_PRU(RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_state)
{
}

/*
   Stop PRU, load firmware (check if firmware is present), start PRU.
   If we get a result the sensor seems to be there.
*/
bool AP_RangeFinder_BBB_PRU::detect()
{
    uint32_t mem_fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
    if(mem_fd == -1) {
        goto FAIL_MEM_FD;
    }

    uint32_t *ctrl = (uint32_t*)mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_CTRL_BASE);
    if(ctrl == nullptr) {
        goto FAIL_MMAP_CTRL;
    }

    void *ram = mmap(0, PRU0_IRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_IRAM_BASE);
    if(ram == nullptr) {
        goto FAIL_MMAP_IRAM;
    }

    // Reset PRU 0
    *ctrl = 0;
    hal.scheduler->delay(1);

    // Load firmware (.text)
    FILE *file = fopen("/lib/firmware/rangefinderprutext.bin", "rb");
    if(file == nullptr) {
        goto FAIL_FOPEN;
    }

    if(fread(ram, PRU0_IRAM_SIZE, 1, file) != 1) {
        goto FAIL_FREAD;
    }

    fclose(file);

    munmap(ram, PRU0_IRAM_SIZE);

    ram = mmap(0, PRU0_DRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_DRAM_BASE);

    // Load firmware (.data)
    file = fopen("/lib/firmware/rangefinderprudata.bin", "rb");
    if(file == nullptr) {
        goto FAIL_FOPEN;
    }

    if(fread(ram, PRU0_DRAM_SIZE, 1, file) != 1) {
        goto FAIL_FREAD;
    }

    fclose(file);

    munmap(ram, PRU0_DRAM_SIZE);

    // Map PRU RAM
    ram = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, PRU0_DRAM_BASE);
    close(mem_fd);

    // Start PRU 0
    *ctrl = 2;

    rangerpru = (volatile struct range*)ram;

    return true;

FAIL_FREAD:	
    fclose(file);

FAIL_FOPEN:
    munmap(ram, PRU0_IRAM_SIZE);

FAIL_MMAP_CTRL:
FAIL_MMAP_IRAM:
    close(mem_fd);

FAIL_MEM_FD:
    return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_BBB_PRU::update(void)
{
    state.status = (RangeFinder::RangeFinder_Status)rangerpru->status;
    state.distance_cm = rangerpru->distance;
}
#endif // CONFIG_HAL_BOARD_SUBTYPE
