#pragma once
//编译底盘板：mingw32-make board-chassis -j4
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
