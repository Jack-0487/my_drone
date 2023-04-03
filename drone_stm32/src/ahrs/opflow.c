

#include "opflow.h"
#include "../bus/uart.h"
#include "../schedule/ticks.h"
#include "altitude.h"
#include "imu.h"
#include "../common/mlib.h"
#include "../fc/decode.h"
#include "../fc/fc.h"
#include <stdio.h>

uint8_t opflowUartBuf[9];
uint8_t opflowUartIndex;

uint16_t opticalflowCtrlTimer, opticalflowMonitorTimer;	
int32_t OpticalflowAltitude = 0; 
float AngleDegreeForOF[2], HeadforOpticalflow = 0;
	
float pixel_flow_distance[2] = {0.0f, 0.0f};
float pixelVel[2];

float optGyro[2] = {0};
float optGyroLast[2] = {0};

uint8_t flagVelCtrl[2] = {0, 0}, opticalFlowEnbAfterStick[2] = {0, 0};		
int32_t opticalflowPID[2] = {0, 0}; 

void opflow_init(void)
{
  opflow_uart_init();
  
  opflowUartIndex = 0;
  optGyro[0] = 0;
  optGyro[1] = 0;
}

void opticalflow_update(void) {  
    uint8_t opflowReady = 0;
    int16_t pixel_flow_integral[2];  
    int16_t pixel_flow_integral_filter[2];
    static int16_t pixel_flow_integral_last[2] = {0};
	
	  static uint16_t lastTimeForPixel = 0;
	  uint16_t currentTimeForPixel;	
    float pixelDelta;
  
    float optSpeed[2];
	
	  uint8_t axis; 
		float AngleDistanceDelta;
			    
    if(opflow_uart_available()) {
      opflowUartBuf[opflowUartIndex] = opflow_uart_read_rx_fifo();
      
      // 0xfe 0x04 d0 d1 d2 d3 sum q 0xaa
      if(opflowUartIndex == 0) {
        if(opflowUartBuf[opflowUartIndex] == 0xfe) opflowUartIndex++;
      } else {
        opflowUartIndex++;
        if(opflowUartIndex >= 9) {
          if(opflowUartBuf[9-1] == 0xaa) {
            opflowReady = 1;
          }          
          opflowUartIndex = 0;
        }
      }
    }
    
	  if(opflowReady) {
        //------------------------------------------------------------------------------------------------------------
		    pixel_flow_integral[0] = (int16_t)(((uint16_t)opflowUartBuf[4]) | ((uint16_t)opflowUartBuf[5] << 8));	
		    pixel_flow_integral[1] = (int16_t)(((uint16_t)opflowUartBuf[2]) | ((uint16_t)opflowUartBuf[3] << 8));	
      
        pixel_flow_integral_filter[0] = (pixel_flow_integral[0] + pixel_flow_integral_last[0]) / 2;
        pixel_flow_integral_filter[1] = (pixel_flow_integral[1] + pixel_flow_integral_last[1]) / 2;
        pixel_flow_integral_last[0] = pixel_flow_integral_filter[0];
        pixel_flow_integral_last[1] = pixel_flow_integral_filter[1];
						
			  pixel_flow_integral[0] = pixel_flow_integral_filter[0];	
		    pixel_flow_integral[1] = pixel_flow_integral_filter[1];
			  opticalflowMonitorTimer = 0;			
      
        //------------------------------------------------------------------------------------------------------------
        optSpeed[0] = pixel_flow_integral[0] * 0.002f;
        optSpeed[1] = pixel_flow_integral[1] * 0.002f;
      
        optGyro[0] = constrain_float(optGyro[0], -0.15f, 0.15f);
        optGyro[1] = constrain_float(optGyro[1], -0.15f, 0.15f);
      
        optGyroLast[0] = (optGyroLast[0] + optGyro[0]) / 2;
        optGyroLast[1] = (optGyroLast[1] + optGyro[1]) / 2;
      
//        printf("%f,%f,%f,%f\n",optSpeed[0]*10, optSpeed[1]*10, optGyro[0]*10, optGyro[1]*10);
//        printf("%f,%f,%f,%f\n",optSpeed[0]*10, optSpeed[1]*10, optGyroLast[0]*10, optGyroLast[1]*10);
              						
        //------------------------------------------------------------------------------------------------------------
			  //2 计算delta、光流摄像头离地高度(mm)
			  currentTimeForPixel = micros16();  
        pixelDelta = (uint16_t)(currentTimeForPixel - lastTimeForPixel)*1e-6;;
			  lastTimeForPixel = currentTimeForPixel;		
 
				OpticalflowAltitude = (OpticalflowAltitude * 3 + max(EstAlt, 250)) / 4;	
				OpticalflowAltitude = constrain(OpticalflowAltitude, 100, 6000); 
									
				//3 计算光流位移及速度
				//  @ 飞行器位移 = 光流位移(测量值) - 机体旋转位移(由陀螺仪积分得到旋转角度)
				//  @ 光流输出量/10000 = 光流摄像头移动(旋转)角度，单位是弧度
				for(axis = 0; axis < 2; axis++) {											
//          optSpeed[axis] = optGyro[axis] - optSpeed[axis];	
          optSpeed[axis] = optGyroLast[axis] - optSpeed[axis];
          optGyro[axis] = 0;

          AngleDistanceDelta = optSpeed[axis] * OpticalflowAltitude;                             
                                                     
          pixel_flow_distance[axis] += AngleDistanceDelta;// * pixelDelta;			
          pixel_flow_distance[axis] = constrain_float(pixel_flow_distance[axis], -3000.0f, 3000.0f);	
          pixelVel[axis] = AngleDistanceDelta / pixelDelta;		

				}		
        
//        printf("%d,%d,%d,%d\n", (int)pixel_flow_distance[0], (int)pixel_flow_distance[1], (int)pixelVel[0], (int)pixelVel[1]);        
//        printf("%d,%d,%d,%d,%d,%d\n", (int)velGlobal[0], (int)velGlobal[1], (int)pixelVel[0], (int)pixelVel[1],(int)pixel_flow_distance[0],(int)pixel_flow_distance[1]);
        
    } 		
}


void getEstimatedPosition(void) {
//    static uint32_t previousT;                                     //Used for update vel & altitude
//    uint32_t currentT;  
    float dt;	 
	  uint16_t axis;
	
		volatile float Pf_Pos;			   
    volatile float Pf_Vel, If_Vel, Df_Vel; 
	                  
	  static int32_t setVelOpticalflow[2] = {0, 0};    	
	  static float VelCurrent[2] = {0.0f}; 
    int32_t error;            
	  static int32_t lastError[2] = {0, 0};
		static float errorOpitalflowVelI[2] = {0, 0};  			
    static int32_t delta1[2] = {0, 0}, delta2[2] = {0, 0}; 
	  int32_t delta, deltaSum;      
    
    static uint16_t stickTimer[2];
						
		//-------------------------------------------------------------------------------------------------    
    dt = 0.01f;
		
    for (axis = 0; axis < 2; axis++) { 
        Pf_Pos = 0.7f;                 
			  Pf_Vel = 1.0f;
        If_Vel = 0.12f;                         
        Df_Vel = 0.01f;    
				
				if(!fc.flags.motorArmed) {
					pixel_flow_distance[axis] = 0;
					pixelVel[axis]            = 0;
					distanceGlobal[axis]      = 0;
					velGlobal[axis]           = 0;
					errorOpitalflowVelI[axis] = 0;
					
					flagVelCtrl[axis]         = 0;
					setVelOpticalflow[axis]   = 0;
				} 		
							
			  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if(rcCommand[axis]!=0 || rcCommand[YAW]!=0 || stickTimer[axis]<100) {
          //----------------------------------------------------------------------------------								
					setVelOpticalflow[axis] = 0;
					pixelVel[axis]          = 0;
					pixel_flow_distance[axis] = 0;					
          velGlobal[axis]         = 0.0f;		
					distanceGlobal[axis]    = 0;
																				
          pixelVel[axis]            = 0;				
          VelCurrent[axis]          = 0;
          errorOpitalflowVelI[axis] = 0;
          lastError[axis] = 0; delta1[axis] = 0; delta2[axis] = 0; 
          
          if(rcCommand[axis]!=0) stickTimer[axis] = 0;
          else stickTimer[axis]++;
          			
				} else {										
          distanceGlobal[axis]   = distanceGlobal[axis]*0.70f + pixel_flow_distance[axis]*0.30f;
          velGlobal[axis]        = velGlobal[axis] * 0.92f + (float)pixelVel[axis] * 0.08f;	
					VelCurrent[axis]       = velGlobal[axis];
									
          setVelOpticalflow[axis] = constrain((int16_t)(-Pf_Pos * distanceGlobal[axis]),  -3000, +3000);
					
				  opticalFlowEnbAfterStick[axis] = 0;		 					
				}			
        
//        printf("%d,%d,%d,%d\n", (int)velGlobal[0], (int)velGlobal[1], (int)distanceGlobal[0], (int)distanceGlobal[1]);

			  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			  //Part3  PID
			  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Velocity PID-Controller
        // P
				#define OPTICALFLOW_PROPORTION_LIMITED     2500   
        error = setVelOpticalflow[axis] - VelCurrent[axis];						  
        opticalflowPID[axis] = constrain((Pf_Vel * error), -OPTICALFLOW_PROPORTION_LIMITED, +OPTICALFLOW_PROPORTION_LIMITED);

        // I 
				#define OPTICALFLOW_INTEGRAL_LIMITED       2000  
				errorOpitalflowVelI[axis] += (If_Vel * error) * dt;
			  if(errorOpitalflowVelI[axis] > OPTICALFLOW_INTEGRAL_LIMITED)  errorOpitalflowVelI[axis] = OPTICALFLOW_INTEGRAL_LIMITED;
			  if(errorOpitalflowVelI[axis] < -OPTICALFLOW_INTEGRAL_LIMITED) errorOpitalflowVelI[axis] = -OPTICALFLOW_INTEGRAL_LIMITED;	
			  opticalflowPID[axis] += errorOpitalflowVelI[axis]; 

        // D	 	
				#define OPTICALFLOW_DIFFERENTIAL_LIMITED   1500  													
				if(dt != 0) delta = (error - lastError[axis]) / dt;      
        else				delta = 0;
        deltaSum          = delta1[axis] + delta2[axis] + delta;
        delta2[axis]      = delta1[axis];
        delta1[axis]      = delta; 
        opticalflowPID[axis] += constrain(deltaSum * Df_Vel, -OPTICALFLOW_DIFFERENTIAL_LIMITED , OPTICALFLOW_DIFFERENTIAL_LIMITED);					
				lastError[axis] = error;
															
    }		
}

void opticalflow_ctrl(void)
{
	//-------------------------------------------------------------------------------
	opticalflow_update();	 

	if (opticalflowCtrlTimer >= 10) {
	  //-----------------------------------------------------------------------------
		opticalflowCtrlTimer = 0;
		getEstimatedPosition();		
//		
//	  //-----------------------------------------------------------------------------
//		//opticalflowMonitorTimer  700ms没光流数据，指示灯持续快闪
//		opticalflowMonitorTimer++;
//		
//	  if(opticalflowMonitorTimer > 70) {        //有串口数据清零    
//		  opticalflowMonitorTimer = 70;
//		  f.bOpticalflowLost = 1;
//	  } else {
//		  f.bOpticalflowLost = 0;
//	  }
//			
//		f.bOpticalflowBad = 0;
	}	
}



