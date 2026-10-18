#include <cstdio>
#include <cstdlib>

#include "rc_protocol.hpp"

namespace {
void check(bool condition, const char* label) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        std::exit(1);
    }
}
} // namespace

int main() {
    RcControl control {};
    SbusData sbus {};
    uint8_t frame[25] {};
    check(!rc_protocol::decodeFrame(nullptr, 25, &control, &sbus), "null buffer");
    check(!rc_protocol::decodeFrame(frame, 17, &control, &sbus), "short frame");
    // Pack DT7 channels using the protocol layout, independently of the decoder.
    const uint16_t dt7[4] = { 364, 1024, 1684, 500 };
    for (int channel = 0; channel < 4; ++channel) {
        for (int bit = 0; bit < 11; ++bit) {
            const int position = channel * 11 + bit;
            frame[position / 8] |= ((dt7[channel] >> bit) & 1) << (position % 8);
        }
    }
    frame[5] |= (RC_SW_DOWN | (RC_SW_MID << 2)) << 4;
    frame[16] = 0;
    frame[17] = 4;
    check(rc_protocol::decodeFrame(frame, 18, &control, &sbus), "DT7 accepted");
    for (int channel = 0; channel < 4; ++channel) {
        check(control.rc.ch[channel] == int(dt7[channel]) - 1024, "DT7 channel offset");
    }
    check(control.rc.s[0] == RC_SW_DOWN && control.rc.s[1] == RC_SW_MID, "DT7 switches");
    check(control.rc.ch[4] == 0, "DT7 roller offset");

#if RC_TYPE != RC_DT7
    for (uint16_t sample = 0; sample < 200; ++sample) {
        uint16_t channels[16];
        for (auto& byte: frame) {
            byte = 0;
        }
        for (int channel = 0; channel < 16; ++channel) {
            channels[channel] = (sample * 37 + channel * 71) & 2047;
            for (int bit = 0; bit < 11; ++bit) {
                const int position = channel * 11 + bit;
                frame[1 + position / 8] |= ((channels[channel] >> bit) & 1) << (position % 8);
            }
        }
    #if RC_TYPE == RC_AT9S_PRO
        frame[23] = AT9S_PRO_RC_CONNECTED_FLAG;
    #elif RC_TYPE == RC_HT8A
        frame[23] = HT8A_RC_CONNECTED_FLAG;
    #else
        frame[23] = ET08A_RC_CONNECTED_FLAG;
    #endif
        check(rc_protocol::decodeFrame(frame, 25, &control, &sbus), "SBUS accepted");
        for (int channel = 0; channel < 16; ++channel) {
            check(sbus.ch[channel] == channels[channel], "SBUS bit packing");
        }
        check(
            control.mouse.x == 0 && control.key.v == 0 && control.rc.ch[4] == 0,
            "non-DT7 special channels cleared"
        );
        for (int index = 0; index < 2; ++index) {
            check(control.rc.s[index] >= 1 && control.rc.s[index] <= 3, "switch index clamped");
        }
    }
#else
    check(
        !rc_protocol::decodeFrame(frame, 25, &control, &sbus),
        "DT7-only configuration rejects SBUS"
    );
#endif
    std::printf("PASS: RC_TYPE=%d decoding and boundary checks\n", RC_TYPE);
}
