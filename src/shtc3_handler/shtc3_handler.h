#ifndef SHTC3_HANDLER_H_
#define SHTC3_HANDLER_H_

void shtc3_fetch(void);
int32_t shtc3_get_temp(void);
uint32_t shtc3_get_humidity(void);
bool init_shtc3(void);

#endif /* SHTC3_HANDLER_H_ */
