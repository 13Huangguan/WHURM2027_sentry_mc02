#pragma once

#include <stdint.h>


//帧格式：SOF | LEN | ID | TYPE | CRC8 | TIMESTAMP | PAYLOAD | CRC16
//        A5 | 1B  | 1B | 1B   | 1B   | 4B        | LEN     | 2B





namespace communication_protocol {

constexpr uint8_t kSof = 0xA5U;          //帧起始字节
constexpr uint8_t kType = 0U;            //

constexpr uint16_t kHeaderSize = 5U;     //帧头总共有 5 个字节,SOF+LEN+ID+TYPE+CRC8(1+1+1+1+1)
constexpr uint16_t kTimestampSize = sizeof(uint32_t);  //时间戳,位于帧头之后
constexpr uint16_t kCrc16Size = sizeof(uint16_t);    //CRC16 校验值是一个 uint16_t，占 2 字节
constexpr uint16_t kFixedOverhead =
    kHeaderSize + kTimestampSize + kCrc16Size;       //无论 Payload 是什么，每一帧都至少固定多出 11 字节
constexpr uint16_t kPayloadOffset = kHeaderSize + kTimestampSize;     //Payload 从第 9 字节开始

constexpr uint16_t kMaxPayloadSize = 128U;
constexpr uint16_t kMaxFrameSize = kFixedOverhead + kMaxPayloadSize;  //最多能保存 139 字节的完整帧

enum class MessageId : uint8_t {           //作用域枚举,避免成员暴露在外部，MessageId 的每个枚举值，在内存中使用 uint8_t 保存，即占 1 字节
    kChassisToGimbal = 0x01,
    kGimbalToChassis = 0x02,
};

struct __attribute__((packed)) FrameHeader {     //__attribute__((packed))：避免有数据插入
    uint8_t sof;
    uint8_t length;
    uint8_t message_id;
    uint8_t type;
    uint8_t crc8;
};

struct Frame {
    uint8_t message_id;                // 消息 ID
    uint32_t timestamp_ms;             // 发送端写入的时间戳
    uint16_t payload_length;           //Payload 实际长度
    uint8_t payload[kMaxPayloadSize];  //Payload 的副本
};


static_assert(sizeof(FrameHeader) == kHeaderSize, "invalid frame header size");
static_assert(kFixedOverhead == 11U, "invalid protocol overhead");


/**
 * @brief 编码一个协议帧。
 * @return 帧总长度，或者在参数无效时为零.
 */
uint16_t encodeFrame(
    MessageId message_id,
    uint32_t timestamp_ms,
    const uint8_t* payload,
    uint8_t payload_length,
    uint8_t* output,
    uint16_t output_capacity
);

/**
 * @brief 增量解包器，处理字节流
 */
class Parser {
public:
    void reset();  //清空解析器
    bool pushByte(uint8_t byte); //向解析器送入一个字节
    bool takeFrame(Frame* frame); //把已经解析完成的帧复制给调用者

private:
    enum class State : uint8_t {
        kWaitSof,
        kReadHeader,
        kReadTimestamp,
        kReadPayload,
        kReadCrc,
    };

    void fail();

    State state_ = State::kWaitSof;
    uint8_t buffer_[kMaxFrameSize] = {};
    uint16_t index_ = 0U;
    uint16_t frame_length_ = 0U;
    Frame frame_ = {};
    bool frame_ready_ = false;
};

} // namespace communication_protocol
