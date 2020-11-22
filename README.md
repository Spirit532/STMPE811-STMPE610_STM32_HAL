# STMPE811.h
A simple single-header STMPE811/STMPE610 driver using HAL with 2-point calibration and averaging
Beware: HAL error handling is not very pretty, can be made better

Usage:
```C++
if(STMPE811_Init(&hi2c2) != HAL_OK) // Init device once
 Error(); // Optionally throw error. If using STMPE610, edit ID on line 276

STMPE811_SetCalData(0, 0, 1, 480, 272, 147, 3962, 261, 3786); // Set calibration parameters for your touch screen(can be updated at any point)

uint16_t tp_x, tp_y;
uint8_t tp_touch;
STMPE811_Read(&hi2c2, &tp_x, &tp_y, &tp_touch); // Can also check for errors
```
