/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h" 
#include "MiniPC.h"


typedef struct{
  //导航接收
uint8_t Header;
  float Vx;
  float Vy;
  float Vz;
	float Spon_Speed;
   uint8_t Vx_test[4];
   uint8_t Vy_test[4];
	 uint8_t Vz_test[4];
	 uint8_t Spin_test[4];
	 uint8_t heat_outpost_int[4];
	 uint8_t Spin_Switch;
  //决策接受
  uint8_t Fire;  //0不开火，1开火
  uint8_t Mode;  //0，自检 1自动
}MiniPc_Receive_Auto_Typedef;


//自瞄接收
typedef struct{
	uint8_t header;
	uint8_t fire_accept;
	float Pitch;
	float Yaw;
  float Distance;
 bool Vision_Grap;
	uint8_t Yaw_test[4];
	uint8_t Pitch_test[4];
	bool Fire_Switch;
	bool Fire_Switch_Spin;
	uint8_t Distance_test[4];
	
	float Fire_Pitch;
	float Fire_Yaw;
	uint8_t Fire_Yaw_test[4];
	uint8_t Fire_Pitch_test[4];
}MiniPc_Receive_Vision_Typedef;


//自瞄发送
typedef struct{
uint8_t Header;

uint8_t *Roll;
uint8_t *Pitch;
uint8_t *Yaw;


}MiniPc_Transmit_Vision_Typedef ;


typedef struct{
struct{
	 uint16_t Stage_Remain_Time;	 //游戏剩余时间
   uint8_t Game_Progress;   //游戏进度
}Game_Status;

struct{
	 uint16_t Current_HP;	 //剩余血量
	 uint16_t Robot_1_HP;
	 uint16_t Robot_3_HP;
	 uint16_t Robot_7_HP;
}Robot_HP;


struct{
uint8_t	Even_Data;
uint8_t Mode_Defense;
}Even_Data;

struct{
	uint8_t Zone_Status;  //补给区状态
	uint8_t Area_Status;  //得分点状态
}Rfid;


}MiniPc_Tx_Auto_Decision_Typedef;



extern MiniPc_Receive_Auto_Typedef MiniPc_Auto;
extern MiniPc_Receive_Vision_Typedef MiniPc_Rx_Vision;
extern MiniPc_Transmit_Vision_Typedef MiniPc_Tx_Vision;
extern MiniPc_Tx_Auto_Decision_Typedef MiniPc_Tx_Auto_Decision;

extern void MiniPC_Transmit_Info(uint8_t *Buff);
extern void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len);
#endif