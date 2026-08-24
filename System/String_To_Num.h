#ifndef __STRING_TO_NUM_H
#define __STRING_TO_NUM_H

#include "stm32f10x.h"

extern int coord[];

/**
 * @brief 从字符串中依次提取无符号十进制数字段，结果写入全局 coord[10]。
 * @param a 以 '\0' 结束的字符串。
 * @return 指向全局 coord 数组的指针。
 * @note 不支持负数和小数，不会自动清空旧 coord，也不检查超过 10 个数字段的越界。
 * @par 使用方法 `uint16_t *v = String_To_Num("X12,Y34");`
 */
uint16_t *String_To_Num(char *a);

#endif
