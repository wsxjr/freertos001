/**
 * @file oled.h
 * @brief SSD1306 OLED 接口
 */
#ifndef __OLED_H
#define __OLED_H

#include "app.h"

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowAngles(const Angles *angles);
void OLED_ShowStatus(void);

#endif /* __OLED_H */
