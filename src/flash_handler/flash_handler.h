#ifndef FLASH_HANDLER_H_
#define FLASH_HANDLER_H_

struct pacer_config_struct{
    uint8_t mode;
    uint8_t test01;
};
bool flash_init(void);

#endif /* FLASH_HANDLER_H_ */
