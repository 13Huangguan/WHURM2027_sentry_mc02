#pragma once

#include "communication_protocol.hpp"

#include "stm32h7xx_hal.h"

#include <stdint.h>

namespace communication {

/**
 * @brief USART10 板间通信设备。
 * @note 接收由 HAL 回调送入软件环形缓冲区，解析和业务处理应在任务上下文中完成。
 */
class Communication {
public:
    /**
     * @brief 初始化板间通信。
     * @return true 表示 USART10 DMA 接收已成功启动。
     * @note 应在 MX_USART10_UART_Init() 之后调用，并只初始化一次。
     */
    bool init();

    /**
     * @brief 编码并通过 USART10 DMA 非阻塞发送一帧。
     * @param message_id 消息类型，必须与当前板角色的发送方向一致。
     * @param timestamp_ms 发送时刻，单位毫秒。
     * @param payload 待发送数据地址；长度为零时可为空。
     * @param payload_length Payload 字节数。
     * @return true 表示 DMA 发送已启动。
     * @note 上一次 DMA 未完成时返回 false，并记录一次发送忙丢帧。
     */
    bool send(
        communication_protocol::MessageId message_id,
        uint32_t timestamp_ms,
        const uint8_t* payload,
        uint8_t payload_length
    );

    /**
     * @brief 从接收缓冲区解析出一帧符合本板方向的数据。
     * @param frame 输出完整协议帧。
     * @param now_ms 当前毫秒时间，用于刷新在线时间。
     * @return true 表示已取得一帧有效数据。
     * @note 通信任务应周期调用；一次调用最多返回一帧。
     */
    bool pollFrame(
        communication_protocol::Frame* frame,
        uint32_t now_ms
    );

    /**
     * @brief 查询设备是否已经成功初始化。
     * @return true 表示接收 DMA 已启动。
     */
    bool isInitialized() const;

    /**
     * @brief 判断对端是否在线。
     * @param now_ms 当前毫秒时间。
     * @return true 表示最近 200 ms 内收到过有效且方向正确的帧。
     */
    bool isOnline(uint32_t now_ms) const;

    /**
     * @brief 获取软件接收环形缓冲区满导致的丢字节次数。
     * @return 累计丢弃字节数。
     */
    uint32_t getRxDropCount() const;

    /**
     * @brief 获取收到了方向不符合当前板角色的完整帧次数。
     * @return 累计异常方向帧数。
     */
    uint32_t getUnexpectedFrameCount() const;

    /**
     * @brief 获取因上一次 DMA 尚未发送完成而放弃发送的次数。
     * @return 累计发送忙丢帧数。
     */
    uint32_t getTxBusyDropCount() const;

    /**
     * @brief 处理 USART10 的 DMA 半传输、传输完成或空闲事件。
     * @param position HAL 返回的循环 DMA 当前写入位置。
     * @note 只能由 HAL_UARTEx_RxEventCallback() 中断上下文调用。
     */
    void processRxEventFromIsr(uint16_t position);

    /**
     * @brief 处理 USART10 DMA 发送完成事件。
     * @note 只能由 HAL_UART_TxCpltCallback() 中断上下文调用。
     */
    void processTxCompleteFromIsr();

    /**
     * @brief 处理 USART10 通信错误并尝试重新启动接收。
     * @note 只能由 HAL_UART_ErrorCallback() 中断上下文调用。
     */
    void processErrorFromIsr();

private:
    /** DMA 接收缓冲区大小，必须不小于单次 DMA 接收窗口。 */
    static constexpr uint16_t kDmaRxBufferSize = 256U;

    /** 软件接收环形缓冲区大小，用于缓存尚未解析的串口字节。 */
    static constexpr uint16_t kRxStreamSize = 512U;

    /** 超过该时间未收到有效帧时判定对端离线，单位毫秒。 */
    static constexpr uint32_t kOfflineTimeoutMs = 200U;

    /**
     * @brief 判断消息是否允许由对端发送给本板。
     * @param message_id 待检查消息 ID。
     * @return true 表示方向符合当前板角色。
     */
    bool isExpectedIncomingMessage(uint8_t message_id) const;

    /**
     * @brief 判断消息是否允许由本板发送。
     * @param message_id 待检查消息 ID。
     * @return true 表示方向符合当前板角色。
     */
    bool isAllowedOutgoingMessage(uint8_t message_id) const;

    /**
     * @brief 将 DMA 缓冲区中的一段字节追加到软件接收环形缓冲区。
     * @param start DMA 缓冲区起始下标。
     * @param length 要读取的字节数。
     * @note 在 USART10 接收中断上下文中调用。
     */
    void pushRxSpan(uint16_t start, uint16_t length);

    /**
     * @brief 向软件接收环形缓冲区推送一个字节。
     * @param byte 接收到的原始字节。
     * @note 缓冲区满时丢弃新字节并增加丢字节计数。
     */
    void pushRxByte(uint8_t byte);

    /** 当前使用的 UART 外设句柄，init() 成功后固定指向 huart10。 */
    UART_HandleTypeDef* uart_ = nullptr;

    /** 协议解析器，任务上下文中通过 pollFrame() 逐字节驱动。 */
    communication_protocol::Parser parser_;

    /**
     * USART10 循环 DMA 接收缓冲区。
     * 32 字节对齐以适配 Cortex-M7 Cache Line，DMA 写入，中断读取。
     */
    alignas(32) uint8_t dma_rx_buffer_[kDmaRxBufferSize] = {};

    /**
     * 已从循环 DMA 缓冲区消费到的位置。
     * 接收中断据此计算本次新增字节区间，只在接收中断中修改。
     */
    uint16_t dma_read_position_ = 0U;

    /** 软件环形缓冲区写指针，由 USART10 接收中断更新。 */
    volatile uint16_t rx_head_ = 0U;

    /** 软件环形缓冲区读指针，由通信任务通过 pollFrame() 更新。 */
    volatile uint16_t rx_tail_ = 0U;

    /** 保存从 DMA 转移到任务的原始串口字节数据。 */
    uint8_t rx_stream_[kRxStreamSize] = {};

    /**
     * USART10 DMA 发送缓冲区。
     * 32 字节对齐以适配 Cortex-M7 Cache Line；DMA 未完成时不得改写。
     */
    alignas(32) uint8_t tx_buffer_[communication_protocol::kMaxFrameSize] = {};

    /** 发送 DMA 忙标志，任务置位，发送完成中断清除。 */
    volatile bool tx_busy_ = false;

    /** 初始化完成标志，表示 USART10 接收 DMA 已经成功启动。 */
    bool initialized_ = false;

    /** 最近一次收到完整有效帧的时间，单位毫秒，用于在线判断。 */
    uint32_t last_valid_rx_ms_ = 0U;

    /** 软件接收环形缓冲区满而丢弃的字节累计数量。 */
    uint32_t rx_drop_count_ = 0U;

    /** 收到方向不符合当前板角色、但仍通过 CRC 的帧累计数量。 */
    uint32_t unexpected_frame_count_ = 0U;

    /** 因上一帧 DMA 尚未发送完成而放弃发送的累计数量。 */
    uint32_t tx_busy_drop_count_ = 0U;
};

extern Communication g_communication;

} // namespace communication
