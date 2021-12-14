/*
 * Copyright (c) 2021 ALueger
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <string.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>
#include <logging/log.h>

#include "flash_handler.h"
#define LOG_MODULE_NAME FLASH_HANDLER
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_NVS_LOG_LEVEL);

static struct nvs_fs fs;

#define STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(storage)
#define FLASH_NODE DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE)

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
#define FLASH_TEST_OFFSET FLASH_AREA_OFFSET(image_1_nonsecure)
#else
#define FLASH_TEST_OFFSET FLASH_AREA_OFFSET(image_1)
#endif

/* 1000 msec = 1 sec */
#define SLEEP_TIME      100
/* maximum reboot counts, make high enough to trigger sector change (buffer */
/* rotation). */
#define MAX_REBOOT 400

#define RBT_CNT_ID 1
#define PACER_CONFIG_ID 2
#define KEY_ID 3
#define STRING_ID 4
#define LONG_ID 5


static struct pacer_config_struct pacer_config;

int rc = 0, cnt = 0, cnt_his = 0;
char buf[16];
uint8_t key[8], longarray[128];
uint32_t reboot_counter = 1U, reboot_counter_his;

struct flash_pages_info info;
const struct device *flash_dev;


bool flash_init(void){
        
	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at FLASH_AREA_OFFSET(storage)
	 */
	printk("Flash Init\n");

	flash_dev = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

	if (!flash_dev) {
		printk("nRF5 flash driver was not found!\n");
		return false;
	}
	
	fs.offset = FLASH_AREA_OFFSET(storage);
	rc = flash_get_page_info_by_offs(flash_dev, fs.offset, &info);
	if (rc) {
		printk("Unable to get page info\n");
		return false;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_init(&fs, flash_dev->name);
	if (rc) {
		printk("Flash Init failed\n");
		return false;
	}
        
        // initial setup Flash Reboot Counter
        rc = nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	if (rc > 0) { /* item was found, show it */
		printk("Id: %d, Read1 Reboot_counter: %d\n",
			RBT_CNT_ID, reboot_counter);
                        reboot_counter= reboot_counter + 1  ;
                        (void)nvs_write(&fs, RBT_CNT_ID, &reboot_counter,sizeof(reboot_counter));

	} else   {/* item was not found, add it */
		printk("No Reboot counter found, adding it at id %d\n",
		       RBT_CNT_ID);
		(void)nvs_write(&fs, RBT_CNT_ID, &reboot_counter,
			  sizeof(reboot_counter));
	}
        
        rc = nvs_read(&fs, PACER_CONFIG_ID, &pacer_config, sizeof(pacer_config));
	if (rc > 0) { /* item was found, show it */
		printk("Id: %d, Read1 Pacer Statusr: %d\n",
			PACER_CONFIG_ID, pacer_config.mode);
                          
                      
	} else   {/* item was not found, add it */
		pacer_config.mode = 1;
                printk("No Pacer Config  found, Set Mode to  %d\n",
		       pacer_config.mode);
                       
		(void)nvs_write(&fs, PACER_CONFIG_ID, &pacer_config,
			  sizeof(pacer_config));
	}


        k_sleep(K_MSEC(100));
        rc = nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	if (rc > 0) { /* item was found, show it */
		printk("Id: %d, Read2 Reboot_counter: %d\n",
			RBT_CNT_ID, reboot_counter);
	} 
        k_sleep(K_MSEC(100));
        rc = nvs_read(&fs,  PACER_CONFIG_ID, &pacer_config, sizeof(pacer_config));
	if (rc > 0) { /* item was found, show it */
		printk("Id: %d, Read Pacer Mode: %d\n",
			 PACER_CONFIG_ID, pacer_config.mode);
	}

        return true;
}



