

#pragma once     

//------------------------------------------------------------------------------------
#include "../platform.h" 


//------------------------------------------------------------------------------------
typedef struct {
  void (*accgyro_init)(void);
  uint8_t (*accgyro_read)(int32_t* acc, int32_t* gyro);
  int32_t accRaw[3];
  int32_t gyroRaw[3];
  int16_t accRaw_offset[3];
  int16_t gyroRaw_offset[3];
  
  uint8_t accCalibrating;
  uint8_t gyroCalibrating;
  
  int32_t accValue[3];        // 传感器读取减offset
  int32_t gyroValue[3];
  
  int32_t accData[3];         // 滤波后
  int32_t gyroData[3];
  
} accgyro_s;


extern accgyro_s accgyro;
extern int16_t acc_1G;
extern float gyroScale;    // gyro output scaled to rad per second

//------------------------------------------------------------------------------------
void accgyro_init(void);
uint8_t accgyro_update(void);

void accgyro_test(void);





