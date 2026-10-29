/**
 * @file app_board_link.cpp
 * @brief 双板应用层通信实现。
 */

#include "app_board_link.hpp"

#include "Board_cfg/board_cfg.hpp"
#include "cmsis_os.h"
#include "dvc_communication.hpp"
#include "lf_core.hpp"
#include "remote_control.h"
#include "usb.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

namespace board_link {
//新增调试对象
TxDebugState g_tx_debug = {};

namespace {


//------------------------------------------------------板内共享状态对象-------------------------------------------
struct SharedState {
    ChassisToGimbalPayload chassis_data = {};
    GimbalToChassisPayload gimbal_data = {};
    uint32_t chassis_rx_ms = 0U;
    uint32_t gimbal_rx_ms = 0U;
    bool has_chassis_data = false;
    bool has_gimbal_data = false;
    Statistics statistics = {};
};

SharedState g_shared_state;
//----------------------------------------------------------------------------------------------------------------
uint32_t g_tx_sequence = 0U;


//------------------------------------------------------发送状态检测-----------------------------------------------
bool hasSequenceGap(uint32_t previous, uint32_t current) {
    return current != (previous + 1U);
}

void recordTxResult(bool success) {
    taskENTER_CRITICAL();
    ++g_shared_state.statistics.tx_attempts;
    ++g_tx_debug.tx_attempts;
    if (success) {
        ++g_tx_debug.tx_successes;
    } else {
        ++g_tx_debug.tx_failures;
    }
    if (!success) {
        ++g_shared_state.statistics.tx_failures;
    }
    taskEXIT_CRITICAL();
}

void recordInvalidRxFrame() {
    taskENTER_CRITICAL();
    ++g_shared_state.statistics.invalid_rx_frames;
    taskEXIT_CRITICAL();
}

bool validateHeader(
    const uint8_t* payload,
    uint16_t payload_length,
    uint16_t expected_length
) {
    if ((payload == nullptr) || (payload_length != expected_length) ||
        (expected_length < 4U)) {
        return false;
    }

    return (payload[0] == kProtocolVersion) &&
           (payload[1] == board_cfg::kConfig.peer_board_id) &&
           (payload[2] == board_cfg::kConfig.self_board_id);
}
//-----------------------------------------------------------------------------------------------------


//------------------------------------------------------发送底盘信息-----------------------------------------------
void sendChassisData(uint32_t now_ms) {
    const LuojiaFox::SystemStateSnapshot state =
        LuojiaFox::system_state.read();
    const RcControl* rc = getRemoteControlPoint();

    ChassisToGimbalPayload payload = {};
    payload.protocol_version = kProtocolVersion;
    payload.sender_board_id = board_cfg::kConfig.self_board_id;
    payload.receiver_board_id = board_cfg::kConfig.peer_board_id;
    payload.sequence = g_tx_sequence++;
    payload.rc_online = getRcOffline() ? 0U : 1U;

    if (rc != nullptr) {
        taskENTER_CRITICAL();
        memcpy(payload.rc_ch, rc->rc.ch, sizeof(payload.rc_ch));
        memcpy(payload.rc_switch, rc->rc.s, sizeof(payload.rc_switch));
        payload.mouse_x = rc->mouse.x;
        payload.mouse_y = rc->mouse.y;
        payload.mouse_z = rc->mouse.z;
        payload.keyboard = rc->key.v;
        payload.mouse_left = rc->mouse.press_l;
        payload.mouse_right = rc->mouse.press_r;
        taskEXIT_CRITICAL();
    }

    payload.vx = state.chassis.vx;
    payload.vy = state.chassis.vy;
    payload.wz = state.chassis.wz;

    payload.imu_yaw_deg = state.imu.yaw_deg;
    payload.imu_pitch_deg = state.imu.pitch_deg;
    payload.imu_roll_deg = state.imu.roll_deg;
    payload.imu_yaw_rate_dps = state.imu.yaw_rate_dps;
    payload.imu_pitch_rate_dps = state.imu.pitch_rate_dps;
    payload.imu_roll_rate_dps = state.imu.roll_rate_dps;

    payload.robot_id = state.referee.robot_id;
    payload.remaining_hp = state.referee.remaining_hp;
    payload.referee_online = state.referee.connected ? 1U : 0U;

    taskENTER_CRITICAL();
    g_tx_debug.last_chassis_payload = payload;
    g_tx_debug.last_tx_ms = now_ms;
    g_tx_debug.last_sequence = payload.sequence;
    g_tx_debug.last_payload_length = sizeof(payload);
    taskEXIT_CRITICAL();

    const bool sent = communication::g_communication.send(
            communication_protocol::MessageId::kChassisToGimbal,
            now_ms,
            reinterpret_cast<const uint8_t*>(&payload),
            sizeof(payload)
        );
    recordTxResult(sent);
}


//------------------------------------------------------发送云台信息-----------------------------------------------
void sendGimbalData(uint32_t now_ms) {
    const LuojiaFox::SystemStateSnapshot state =
        LuojiaFox::system_state.read();
    const bool usb_online = !usb.IsOffline();

    GimbalToChassisPayload payload = {};
    payload.protocol_version = kProtocolVersion;
    payload.sender_board_id = board_cfg::kConfig.self_board_id;
    payload.receiver_board_id = board_cfg::kConfig.peer_board_id;
    payload.sequence = g_tx_sequence++;

    if (usb_online) {
        payload.vx = state.robot_cmd.vx;
        payload.vy = state.robot_cmd.vy;
        payload.wz = state.robot_cmd.wz;
    }

    payload.usb_online = usb_online ? 1U : 0U;

    taskENTER_CRITICAL();
    g_tx_debug.last_gimbal_payload = payload;
    g_tx_debug.last_tx_ms = now_ms;
    g_tx_debug.last_sequence = payload.sequence;
    g_tx_debug.last_payload_length = sizeof(payload);
    taskEXIT_CRITICAL();

    const bool sent = communication::g_communication.send(
            communication_protocol::MessageId::kGimbalToChassis,
            now_ms,
            reinterpret_cast<const uint8_t*>(&payload),
            sizeof(payload)
        );
    recordTxResult(sent);
}


//------------------------------------------------------更新底盘状态------------------------------------------------
void publishChassisRemoteState(
    const ChassisToGimbalPayload& payload,
    uint32_t now_ms
) {
    LuojiaFox::system_state.updateChassis({
        payload.vx,
        payload.vy,
        payload.wz
    });

    LuojiaFox::system_state.updateReferee({
        payload.robot_id,
        payload.remaining_hp,
        payload.referee_online != 0U
    });

    taskENTER_CRITICAL();
    const bool has_previous = g_shared_state.has_chassis_data;
    const uint32_t previous_sequence =
        g_shared_state.chassis_data.sequence;
    g_shared_state.chassis_data = payload;
    g_shared_state.chassis_rx_ms = now_ms;
    g_shared_state.has_chassis_data = true;
    ++g_shared_state.statistics.valid_rx_frames;
    if (has_previous &&
        hasSequenceGap(previous_sequence, payload.sequence)) {
        ++g_shared_state.statistics.sequence_gaps;
    }
    taskEXIT_CRITICAL();
}


//------------------------------------------------------更新云台状态------------------------------------------------
void publishGimbalRemoteState(
    const GimbalToChassisPayload& payload,
    uint32_t now_ms
) {
    LuojiaFox::system_state.updateRobotCmd({
        payload.vx,
        payload.vy,
        payload.wz,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0U,
        0U,
        now_ms
    });

    taskENTER_CRITICAL();
    const bool has_previous = g_shared_state.has_gimbal_data;
    const uint32_t previous_sequence =
        g_shared_state.gimbal_data.sequence;
    g_shared_state.gimbal_data = payload;
    g_shared_state.gimbal_rx_ms = now_ms;
    g_shared_state.has_gimbal_data = true;
    ++g_shared_state.statistics.valid_rx_frames;
    if (has_previous &&
        hasSequenceGap(previous_sequence, payload.sequence)) {
        ++g_shared_state.statistics.sequence_gaps;
    }
    taskEXIT_CRITICAL();
}


//-------------------------------------------------------通信帧分发-------------------------------------------------
void processIncomingFrame(
    const communication_protocol::Frame& frame,
    uint32_t now_ms
) {
    if (board_cfg::kIsChassisBoard) {
        if (!validateHeader(
                frame.payload,
                frame.payload_length,
                sizeof(GimbalToChassisPayload)
            )) {
            recordInvalidRxFrame();
            return;
        }

        GimbalToChassisPayload payload = {};
        memcpy(&payload, frame.payload, sizeof(payload));
        publishGimbalRemoteState(payload, now_ms);
        return;
    }

    if (!validateHeader(
            frame.payload,
            frame.payload_length,
            sizeof(ChassisToGimbalPayload)
        )) {
        recordInvalidRxFrame();
        return;
    }

    ChassisToGimbalPayload payload = {};
    memcpy(&payload, frame.payload, sizeof(payload));
    publishChassisRemoteState(payload, now_ms);
}


//--------------------------------------------------发送本板信息------------------------------------------------
void sendLocalState(uint32_t now_ms) {
    if (board_cfg::kIsChassisBoard) {
        sendChassisData(now_ms);
    } else {
        sendGimbalData(now_ms);
    }
}

} // namespace

bool getRemoteChassisData(ChassisToGimbalPayload* output) {
    if (output == nullptr) {
        return false;
    }

    taskENTER_CRITICAL();
    const bool available = g_shared_state.has_chassis_data;
    if (available) {
        *output = g_shared_state.chassis_data;
    }
    taskEXIT_CRITICAL();
    return available;
}

bool getRemoteGimbalData(GimbalToChassisPayload* output) {
    if (output == nullptr) {
        return false;
    }

    taskENTER_CRITICAL();
    const bool available = g_shared_state.has_gimbal_data;
    if (available) {
        *output = g_shared_state.gimbal_data;
    }
    taskEXIT_CRITICAL();
    return available;
}


//------------------------------------------------------对端在线检测-----------------------------------------------
bool isPeerOnline(uint32_t now_ms) {
    taskENTER_CRITICAL();
    const bool has_data = board_cfg::kIsChassisBoard
        ? g_shared_state.has_gimbal_data
        : g_shared_state.has_chassis_data;
    const uint32_t last_rx_ms = board_cfg::kIsChassisBoard
        ? g_shared_state.gimbal_rx_ms
        : g_shared_state.chassis_rx_ms;
    taskEXIT_CRITICAL();

    return has_data &&
           ((now_ms - last_rx_ms) < board_cfg::kConfig.link_timeout_ms);
}

Statistics getStatistics() {
    taskENTER_CRITICAL();
    const Statistics result = g_shared_state.statistics;
    taskEXIT_CRITICAL();
    return result;
}

} // namespace board_link




//------------------------------------------------------板间通信任务-----------------------------------------------
extern "C" void BoardLink_Task(void const* argument) {
    (void)argument;

    uint32_t last_init_attempt_ms = 0U;
    uint32_t last_tx_ms = 0U;
    bool initialized = false;
    bool init_attempted = false;
    bool tx_started = false;  //tx_started 的作用是保证第一次进入该分支时立即发送一帧，不用等待 10 ms

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        if ((!initialized) &&
            ((!init_attempted) ||
             ((now_ms - last_init_attempt_ms) >= 1000U))) {
            last_init_attempt_ms = now_ms;
            init_attempted = true;
            initialized = communication::g_communication.init();
        }

        if (initialized) {
            for (;;) {
                communication_protocol::Frame frame = {};
                if (!communication::g_communication.pollFrame(
                        &frame,
                        now_ms
                    )) {
                    break;
                }
                board_link::processIncomingFrame(frame, now_ms);
            }

            if ((!tx_started) ||
                ((now_ms - last_tx_ms) >= board_link::kTxPeriodMs)) {
                board_link::sendLocalState(now_ms);
                last_tx_ms = now_ms;
                tx_started = true;
            }
        }

        osDelay(board_link::kTaskPeriodMs);
    }
}
