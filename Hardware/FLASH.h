#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f10x.h"

#define FLASH_Page_Size  (0x00000400) // 1 KB 页大小
#define FLASH_Start_Addr (0x08007c00) // 固定读写页起始地址
#define FLASH_End_Addr   (0x08007fff) // 固定读写页结束地址
#define DATA_32          (0x12345678)

/**
 * @brief 从固定 Flash 地址 0x08007C00 连续读取 int16_t 数据。
 * @param data 接收数组，不能为 NULL。
 * @param len 读取的 int16_t 个数，最大应为 512。
 * @note 不检查越界，也不验证数据有效性。
 * @par 使用方法 `FLASH_read(MpuOffset, 6);`
 */
extern void FLASH_read(int16_t *data, uint8_t len);

/**
 * @brief 擦除固定 Flash 页后，从 0x08007C00 连续写入 int16_t 数据。
 * @param data 待写数组，不能为 NULL。
 * @param len 写入的 int16_t 个数，最大应为 512。
 * @note 每次调用都会擦除整页；擦除或写入失败时当前实现会永久停在 while(1)。
 * @par 使用方法 `FLASH_write(MpuOffset, 6);`
 */
extern void FLASH_write(int16_t *data, uint8_t len);

#endif
