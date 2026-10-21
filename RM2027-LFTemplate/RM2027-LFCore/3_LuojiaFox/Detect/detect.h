#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 所有需要检测的设备 */
typedef enum {
    DETECT_RC = 0,
    DETECT_PITCH_MOTOR,
    DETECT_WHEEL1,
    DETECT_WHEEL2,
    DETECT_WHEEL3,
    DETECT_WHEEL4,
    DETECT_BMI_GYRO,
    DETECT_BMI_ACCEL,
    DETECT_NUC,
    DETECT_COUNT
} detect_id_t;  //每个检测对象唯一的编号

/* 每个设备的检测状态 */
typedef struct {
    volatile uint32_t new_time;    //最近一次收到有效数据的时间
    volatile uint32_t last_time;   //上一次收到有效数据的时间，用来计算频率
    volatile uint32_t lost_time;   //开始判定掉线的时间
    uint32_t work_time;            //设备刚恢复上线时，用于稳定观察

    uint32_t offline_time_ms;      //超过多久没收到数据算掉线
    uint32_t online_time_ms;       //恢复后观察多久才认为真正稳定

    uint8_t priority;              
    uint8_t enable;

    bool error_exist;              //设备当前是否存在错误
    bool is_lost;                  //是否掉线
    bool data_is_error;            //数据能收到，但数值可能异常

    float frequency;               //频率

    bool (*data_is_error_fun)(void);          //函数指针，不同设备的检查函数
    void (*solve_lost_fun)(void);             //掉线后的保护动作，例如重启 DMA
    void (*solve_data_error_fun)(void);       //数据异常后的保护动作
} error_t;

//toe 可以理解为“设备编号”或“设备槽位”

void detect_hook(uint8_t toe);             //告诉检测模块，某个设备刚刚收到了有效数据
bool toe_is_error(uint8_t toe);            //查询某个设备当前是否存在错误
const error_t *get_error_list_point(void); //返回整个错误状态数组的首地址

#ifdef __cplusplus
}
#endif

#endif