#include "communication_protocol.hpp"

#include "CRC8_CRC16.h"

#include <string.h>

namespace communication_protocol {

uint16_t encodeFrame(
    MessageId message_id,
    uint32_t timestamp_ms,
    const uint8_t* payload,
    uint8_t payload_length,
    uint8_t* output,
    uint16_t output_capacity
) {
    if ((output == nullptr) || (payload_length > kMaxPayloadSize) ||
        ((payload == nullptr) && (payload_length != 0U))) {
        return 0U;
    }

    const uint16_t frame_length =
        static_cast<uint16_t>(kFixedOverhead + payload_length);  //C++ 的显式类型转换语法

    if (output_capacity < frame_length) {
        return 0U;
    }

    output[0] = kSof;
    output[1] = payload_length;
    output[2] = static_cast<uint8_t>(message_id);
    output[3] = kType;
    output[4] = 0U;

    append_CRC8_check_sum(output, kHeaderSize);

    output[5] = static_cast<uint8_t>(timestamp_ms);        //采用小端序，多字节整数中，低位字节先存放或先发送，高位字节后存放或后发送
    output[6] = static_cast<uint8_t>(timestamp_ms >> 8U);
    output[7] = static_cast<uint8_t>(timestamp_ms >> 16U);
    output[8] = static_cast<uint8_t>(timestamp_ms >> 24U);

    if (payload_length != 0U) {
        memcpy(output + kPayloadOffset, payload, payload_length);
    }

    append_CRC16_check_sum(output, frame_length);
    return frame_length;
}

void Parser::reset() {
    state_ = State::kWaitSof;
    index_ = 0U;
    frame_length_ = 0U;
    frame_ready_ = false;
}

void Parser::fail() {
    state_ = State::kWaitSof;
    index_ = 0U;
    frame_length_ = 0U;
}

bool Parser::pushByte(uint8_t byte) {
    if (frame_ready_) {
        return false;
    }

    switch (state_) {
        case State::kWaitSof:
            if (byte == kSof) {
                index_ = 0U;
                buffer_[index_++] = byte;
                state_ = State::kReadHeader;
            }
            return false;

        case State::kReadHeader:
            buffer_[index_++] = byte;

            if (index_ < kHeaderSize) {
                return false;
            }

            if ((buffer_[1] > kMaxPayloadSize) || (buffer_[3] != kType) ||
                (!verify_CRC8_check_sum(buffer_, kHeaderSize))) {
                fail();
                return false;
            }

            frame_length_ =
                static_cast<uint16_t>(kFixedOverhead + buffer_[1]);
            state_ = State::kReadTimestamp;
            return false;

        case State::kReadTimestamp:
            buffer_[index_++] = byte;

            if (index_ >= kPayloadOffset) {
                state_ =
                    buffer_[1] == 0U ? State::kReadCrc : State::kReadPayload;
            }
            return false;

        case State::kReadPayload:
            buffer_[index_++] = byte;

            if (index_ >=
                static_cast<uint16_t>(kPayloadOffset + buffer_[1])) {
                state_ = State::kReadCrc;
            }
            return false;

        case State::kReadCrc:
            buffer_[index_++] = byte;

            if (index_ < frame_length_) {
                return false;
            }

            if (!verify_CRC16_check_sum(buffer_, frame_length_)) {
                fail();
                return false;
            }

            frame_.message_id = buffer_[2];
            frame_.payload_length = buffer_[1];
            frame_.timestamp_ms =
                static_cast<uint32_t>(buffer_[5]) |
                (static_cast<uint32_t>(buffer_[6]) << 8U) |
                (static_cast<uint32_t>(buffer_[7]) << 16U) |
                (static_cast<uint32_t>(buffer_[8]) << 24U);

            if (frame_.payload_length != 0U) {
                memcpy(
                    frame_.payload,
                    buffer_ + kPayloadOffset,
                    frame_.payload_length
                );
            }

            frame_ready_ = true;
            state_ = State::kWaitSof;
            index_ = 0U;
            frame_length_ = 0U;
            return true;
    }

    fail();
    return false;
}

bool Parser::takeFrame(Frame* frame) {
    if ((frame == nullptr) || (!frame_ready_)) {
        return false;
    }

    *frame = frame_;
    frame_ready_ = false;
    return true;
}

} // namespace communication_protocol
