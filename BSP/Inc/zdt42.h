#ifndef __ZDT42_H
#define __ZDT42_H
#include <stdint.h>

void ZDT42_Init(void);
void ZDT42_Enable(uint8_t id, uint8_t enable);
void ZDT42_Stop(uint8_t id);
void ZDT42_SetSpeed(uint8_t id, int16_t rpm, uint16_t accel);
void ZDT42_MoveRelative(uint8_t id, float angle_deg, uint16_t rpm, uint8_t accel);
void ZDT42_MoveAbsolute(uint8_t id, float angle_deg, uint16_t rpm, uint8_t accel);
float ZDT42_ReadPosition(uint8_t id);
int16_t ZDT42_ReadSpeed(uint8_t id);

#endif