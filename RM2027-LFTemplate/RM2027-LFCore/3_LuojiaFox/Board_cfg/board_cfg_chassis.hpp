#pragma once
//编译底盘板：mingw32-make board-chassis -j4
//查看云台发来的消息：g_shared_state.gimbal_data
//查看底盘(本板)最近一次发送的消息：g_tx_debug
namespace board_cfg {

constexpr RoleConfig kConfig = {
    BoardRole::kChassisYaw, // role

    9U, // self_board_id
    10U, // peer_board_id

    true, // has_chassis
    true, // has_yaw
    false, // has_pitch
    false, // has_shooter

    true, // has_rc_receiver
    true, // has_referee
    false, // has_mini_pc

    RcSource::kDirect, // rc_source
    200U, // link_timeout_ms
};

} // namespace board_cfg
