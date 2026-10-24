#pragma once

#include <stdint.h>

namespace board_cfg {

enum class BoardRole : uint8_t {
    kChassisYaw = 1,
    kAimGimbal = 2,
};

enum class RcSource : uint8_t {
    kNone = 0,
    kDirect = 1,
    kBoardLink = 2,
};

struct RoleConfig {
    BoardRole role;

    uint8_t self_board_id;
    uint8_t peer_board_id;

    bool has_chassis;
    bool has_yaw;
    bool has_pitch;
    bool has_shooter;

    bool has_rc_receiver;
    bool has_referee;
    bool has_mini_pc;

    RcSource rc_source;
    uint32_t link_timeout_ms;
};

} // namespace board_cfg

#ifndef BOARD_ROLE
#error "BOARD_ROLE is not defined: use BOARD_ROLE=1 or BOARD_ROLE=2"
#endif

#if BOARD_ROLE == 1
#include "board_cfg_chassis.hpp"
#elif BOARD_ROLE == 2
#include "board_cfg_gimbal.hpp"
#else
#error "Invalid BOARD_ROLE: expected 1 or 2"
#endif

namespace board_cfg {

static_assert(
    static_cast<uint8_t>(kConfig.role) == BOARD_ROLE,
    "BOARD_ROLE does not match the selected board configuration"
);

static_assert(
    kConfig.self_board_id != kConfig.peer_board_id,
    "self and peer board IDs must differ"
);

static_assert(
    kConfig.link_timeout_ms > 0U,
    "board link timeout must be positive"
);

constexpr bool kIsChassisBoard = kConfig.role == BoardRole::kChassisYaw;
constexpr bool kIsGimbalBoard = kConfig.role == BoardRole::kAimGimbal;

} // namespace board_cfg
