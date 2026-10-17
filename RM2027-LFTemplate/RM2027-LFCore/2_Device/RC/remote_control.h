
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
// clang-format off
#include <stdbool.h>

#include "drv_uart.h"
//#include "struct_typedef.h"
//#include "attribute_typedef.h"
//#include "macro_typedef.h"

#define RC_DT7      0  // DT7遥控器
#define RC_AT9S_PRO 1  // AT9S PRO遥控器
#define RC_HT8A     2  // HT8A遥控器
#define RC_ET08A    3  // ET08A遥控器


#ifndef __RC_TYPE
#define __RC_TYPE RC_ET08A  // 遥控器类型

#endif
#ifndef __CONTROL_LINK_RC
#define __CONTROL_LINK_RC CL_RC_DIRECT  // 遥控器数据传输方式
#endif

#define AX_X 	0  // x轴
#define AX_Y 	1  // y轴
#define AX_Z 	2  // z轴

#define AX_ROLL AX_X    // roll轴
#define AX_PITCH AX_Y   // pitch轴
#define AX_YAW AX_Z     // yaw轴

/*遥控器相关宏定义*/

#define DT7_SW_LEFT 1 // DT7遥控器左侧拨杆通道号
#define DT7_SW_RIGHT 0 // DT7遥控器右侧拨杆通道号
#define DT7_CH_LH 0 // DT7遥控器右侧水平摇杆通道号
#define DT7_CH_LV 1 // DT7遥控器右侧竖直摇杆通道号
#define DT7_CH_RH 2 // DT7遥控器左侧水平摇杆通道号
#define DT7_CH_RV 3 // DT7遥控器左侧竖直摇杆通道号
#define DT7_CH_ROLLER 4 // DT7遥控器左上角滚轮通道号

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u
#define SBUS_RC_FRAME_LENGTH 25u
// DT7遥控器通道值范围
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

// AT9S PRO 遥控器通道值范围
#define AT9S_PRO_RC_CH_VALUE_MIN         ((uint16_t)200)
#define AT9S_PRO_RC_CH_VALUE_OFFSET      ((uint16_t)1000)
#define AT9S_PRO_RC_CH_VALUE_MAX         ((uint16_t)1800)

#define AT9S_PRO_RC_CONNECTED_FLAG       ((uint8_t)12)

// HT8A 遥控器通道值范围
#define HT8A_RC_CH013_VALUE_MIN         ((uint16_t)432)
#define HT8A_RC_CH013_VALUE_OFFSET      ((uint16_t)992)
#define HT8A_RC_CH013_VALUE_MAX         ((uint16_t)1552)

#define HT8A_RC_CH247_VALUE_MIN         ((uint16_t)192)
#define HT8A_RC_CH247_VALUE_OFFSET      ((uint16_t)992)
#define HT8A_RC_CH247_VALUE_MAX         ((uint16_t)1792)

#define HT8A_RC_CH47_VALUE_MIN         ((uint16_t)192)
#define HT8A_RC_CH47_VALUE_OFFSET      ((uint16_t)992)
#define HT8A_RC_CH47_VALUE_MAX         ((uint16_t)1792)

#define HT8A_RC_CONNECTED_FLAG         ((uint8_t)12)

// ET08A 遥控器通道值范围
#define ET08A_RC_CH_VALUE_MIN         ((uint16_t)353)
#define ET08A_RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define ET08A_RC_CH_VALUE_MAX         ((uint16_t)1694)

#define ET08A_RC_CONNECTED_FLAG       ((uint8_t)3)


#define RC_TO_ONE 0.0015151515151515f  // (1/660)遥控器通道值归一化系数

// 遥控器掉线时间阈值
#define RC_LOST_TIME 100  // ms
// 非dt7遥控器连续断线上线次数（超过认为断连）
#define SBUS_MAX_LOST_NUN 10
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define RC_CH_LEFT_HORIZONTAL   ((uint8_t)2)
#define RC_CH_LEFT_VERTICAL     ((uint8_t)3)
#define RC_CH_RIGHT_HORIZONTAL  ((uint8_t)0)
#define RC_CH_RIGHT_VERTICAL    ((uint8_t)1)
#define RC_CH_LEFT_ROTATE       ((uint8_t)4)

#define RC_SW_LEFT              ((uint8_t)1)
#define RC_SW_RIGHT             ((uint8_t)0)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W     ((uint8_t)0)
#define KEY_S     ((uint8_t)1)
#define KEY_A     ((uint8_t)2)
#define KEY_D     ((uint8_t)3)
#define KEY_SHIFT ((uint8_t)4)
#define KEY_CTRL  ((uint8_t)5)
#define KEY_Q     ((uint8_t)6)
#define KEY_E     ((uint8_t)7)
#define KEY_R     ((uint8_t)8)
#define KEY_F     ((uint8_t)9)
#define KEY_G     ((uint8_t)10)
#define KEY_Z     ((uint8_t)11)
#define KEY_X     ((uint8_t)12)
#define KEY_C     ((uint8_t)13)
#define KEY_V     ((uint8_t)14)
#define KEY_B     ((uint8_t)15)

#define KEY_LEFT  ((uint8_t)0)
#define KEY_RIGHT ((uint8_t)1)

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << KEY_W)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << KEY_S)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << KEY_A)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << KEY_D)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << KEY_SHIFT)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << KEY_CTRL)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << KEY_Q)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << KEY_E)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << KEY_R)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << KEY_F)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << KEY_G)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << KEY_Z)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << KEY_X)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << KEY_C)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << KEY_V)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << KEY_B)

/* ----------------------- Data Struct ------------------------------------- */
typedef struct __RC_ctrl
{
        struct __rc
        {
                int16_t ch[5];
                char s[4];
        } __attribute__((packed)) rc;
        struct __mouse
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } __attribute__((packed)) mouse;
        struct __key
        {
                uint16_t v;
        } __attribute__((packed)) key;

} __attribute__((packed)) RC_ctrl_t;

typedef struct
{
        uint16_t ch[16];
        uint8_t connect_flag;
} __attribute__((packed)) Sbus_t;
// clang-format on

/* ----------------------- Internal Data ----------------------------------- */

#ifdef __cplusplus

class RemoteControl
{
public:
    /* 遥控器控制变量 */
    RC_ctrl_t rc_ctrl;
    Sbus_t sbus;

    /* 生命周期 */
    void Init(void);

    /* 数据查询 */
    const RC_ctrl_t *GetCtrlPoint(void);
    const Sbus_t *GetSbusPoint(void);
    uint8_t DataIsError(void);
    void SolveRcLost(void);
    void SolveDataError(void);

    /* 串口中断的实际处理逻辑 */
    void OnUart5Irq(void);

    /******************************************************************/
    /* API                                                            */
    /*----------------------------------------------------------------*/
    /* function:      IsOffline                                       */
    /*                GetRcCh                                         */
    /*                GetRcSw                                         */
    /*                GetMouseSpeed                                   */
    /*                GetMouse                                        */
    /*                GetKeyboard                                     */
    /******************************************************************/
    bool IsOffline(void);
    float GetRcCh(uint8_t ch);
    char GetRcSw(uint8_t sw);
    float GetMouseSpeed(uint8_t axis);
    bool GetMouse(uint8_t key);
    bool GetKeyboard(uint8_t key);

private:
    /* 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界 */
    uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

    /* 上一次接收数据的时间 */
    uint32_t last_receive_time = 0;
    /* 记录连续接收数据的次数 */
    uint32_t receive_count = 0;
    /* 记录非dt7的sbus遥控器连续断连次数 */
    uint32_t sbus_lost_count = SBUS_MAX_LOST_NUN + 5;

#if (__RC_TYPE != RC_DT7)
    uint8_t connected_flag = 0;  /* 遥控器连接标志位 */
#endif  // __RC_TYPE != RC_DT7

    /* 取正函数 */
    static int16_t RC_abs(int16_t value);

    void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#if (__RC_TYPE == RC_AT9S_PRO)
    void At9sProSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
    void Ht8aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
    void Et08aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#endif
};

extern RemoteControl remote_control;

#endif  /* __cplusplus */

/* main.c 和中断向量表按符号名引用，必须保持 C 链接 */
#ifdef __cplusplus
extern "C" {
#endif
extern void remote_control_init(void);
extern void UART5_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif
/*------------------------------ End of File ------------------------------*/
