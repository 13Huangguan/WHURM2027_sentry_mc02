#include "dvc_communication.hpp"

#include "Board_cfg/board_cfg.hpp"
#include "usart.h"

#include <string.h>

namespace communication {

Communication g_communication;

bool Communication::init() {
    uart_ = &huart10;

    parser_.reset();
    rx_head_ = 0U;
    rx_tail_ = 0U;
    dma_read_position_ = 0U;
    rx_drop_count_ = 0U;
    unexpected_frame_count_ = 0U;
    tx_busy_drop_count_ = 0U;
    tx_busy_ = false;
    last_valid_rx_ms_ = 0U;
    initialized_ = false;

    memset(dma_rx_buffer_, 0, sizeof(dma_rx_buffer_));
    memset(rx_stream_, 0, sizeof(rx_stream_));
    memset(tx_buffer_, 0, sizeof(tx_buffer_));

    SCB_CleanInvalidateDCache_by_Addr(
        reinterpret_cast<uint32_t*>(dma_rx_buffer_),
        sizeof(dma_rx_buffer_)
    );

    (void)HAL_UART_AbortReceive(uart_);

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        uart_,
        dma_rx_buffer_,
        sizeof(dma_rx_buffer_)
    );

    if (status != HAL_OK) {
        (void)HAL_UART_AbortReceive(uart_);
        status = HAL_UARTEx_ReceiveToIdle_DMA(
            uart_,
            dma_rx_buffer_,
            sizeof(dma_rx_buffer_)
        );
    }

    if (status != HAL_OK) {
        uart_ = nullptr;
        return false;
    }

    initialized_ = true;
    return true;
}

bool Communication::send(
    communication_protocol::MessageId message_id,
    uint32_t timestamp_ms,
    const uint8_t* payload,
    uint8_t payload_length
) {
    if ((uart_ == nullptr) || (!initialized_)) {
        return false;
    }

    if (!isAllowedOutgoingMessage(static_cast<uint8_t>(message_id))) {
        return false;
    }

    if (tx_busy_) {
        ++tx_busy_drop_count_;
        return false;
    }

    const uint16_t frame_length = communication_protocol::encodeFrame(
        message_id,
        timestamp_ms,
        payload,
        payload_length,
        tx_buffer_,
        sizeof(tx_buffer_)
    );

    if (frame_length == 0U) {
        return false;
    }

    tx_busy_ = true;

    SCB_CleanDCache_by_Addr(
        reinterpret_cast<uint32_t*>(tx_buffer_),
        frame_length
    );

    const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        uart_,
        tx_buffer_,
        frame_length
    );

    if (status != HAL_OK) {
        tx_busy_ = false;
        return false;
    }

    return true;
}

bool Communication::pollFrame(
    communication_protocol::Frame* frame,
    uint32_t now_ms
) {
    if ((frame == nullptr) || (!initialized_)) {
        return false;
    }

    if (parser_.takeFrame(frame)) {
        last_valid_rx_ms_ = now_ms;
        return true;
    }

    for (;;) {
        while (rx_tail_ != rx_head_) {
            const uint8_t byte = rx_stream_[rx_tail_];
            rx_tail_ = static_cast<uint16_t>(
                (rx_tail_ + 1U) % kRxStreamSize
            );

            if (!parser_.pushByte(byte)) {
                continue;
            }

            if (!parser_.takeFrame(frame)) {
                continue;
            }

            if (!isExpectedIncomingMessage(frame->message_id)) {
                ++unexpected_frame_count_;
                continue;
            }

            last_valid_rx_ms_ = now_ms;
            return true;
        }

        __DMB();
        if (rx_tail_ == rx_head_) {
            break;
        }
    }

    return false;
}

bool Communication::isInitialized() const {
    return initialized_;
}

bool Communication::isOnline(uint32_t now_ms) const {
    if ((!initialized_) || (last_valid_rx_ms_ == 0U)) {
        return false;
    }

    return (now_ms - last_valid_rx_ms_) < kOfflineTimeoutMs;
}

uint32_t Communication::getRxDropCount() const {
    return rx_drop_count_;
}

uint32_t Communication::getUnexpectedFrameCount() const {
    return unexpected_frame_count_;
}

uint32_t Communication::getTxBusyDropCount() const {
    return tx_busy_drop_count_;
}

bool Communication::isExpectedIncomingMessage(uint8_t message_id) const {
    if (board_cfg::kIsChassisBoard) {
        return message_id == static_cast<uint8_t>(
            communication_protocol::MessageId::kGimbalToChassis
        );
    }

    return message_id == static_cast<uint8_t>(
        communication_protocol::MessageId::kChassisToGimbal
    );
}

bool Communication::isAllowedOutgoingMessage(uint8_t message_id) const {
    if (board_cfg::kIsChassisBoard) {
        return message_id == static_cast<uint8_t>(
            communication_protocol::MessageId::kChassisToGimbal
        );
    }

    return message_id == static_cast<uint8_t>(
        communication_protocol::MessageId::kGimbalToChassis
    );
}

void Communication::pushRxByte(uint8_t byte) {
    const uint16_t next =
        static_cast<uint16_t>((rx_head_ + 1U) % kRxStreamSize);

    if (next == rx_tail_) {
        ++rx_drop_count_;
        return;
    }

    rx_stream_[rx_head_] = byte;
    __DMB();
    rx_head_ = next;
}

void Communication::pushRxSpan(uint16_t start, uint16_t length) {
    if (start >= kDmaRxBufferSize) {
        return;
    }

    const uint16_t available =
        static_cast<uint16_t>(kDmaRxBufferSize - start);
    const uint16_t copy_length =
        length < available ? length : available;

    for (uint16_t index = 0U; index < copy_length; ++index) {
        pushRxByte(dma_rx_buffer_[start + index]);
    }
}

void Communication::processRxEventFromIsr(uint16_t position) {
    if ((uart_ == nullptr) || (!initialized_)) {
        return;
    }

    if (HAL_UARTEx_GetRxEventType(uart_) == HAL_UART_RXEVENT_HT) {
        return;
    }

    uint16_t current = position;
    if (current > kDmaRxBufferSize) {
        current = kDmaRxBufferSize;
    }

    SCB_InvalidateDCache_by_Addr(
        reinterpret_cast<uint32_t*>(dma_rx_buffer_),
        sizeof(dma_rx_buffer_)
    );

    const HAL_UART_RxEventTypeTypeDef event =
        HAL_UARTEx_GetRxEventType(uart_);

    if (event == HAL_UART_RXEVENT_TC) {
        if (dma_read_position_ < kDmaRxBufferSize) {
            pushRxSpan(
                dma_read_position_,
                static_cast<uint16_t>(
                    kDmaRxBufferSize - dma_read_position_
                )
            );
        }
        dma_read_position_ = 0U;
        return;
    }

    if (event != HAL_UART_RXEVENT_IDLE) {
        return;
    }

    if (current >= dma_read_position_) {
        pushRxSpan(
            dma_read_position_,
            static_cast<uint16_t>(current - dma_read_position_)
        );
    } else {
        pushRxSpan(
            dma_read_position_,
            static_cast<uint16_t>(
                kDmaRxBufferSize - dma_read_position_
            )
        );
        pushRxSpan(0U, current);
    }

    dma_read_position_ = current;
}

void Communication::processTxCompleteFromIsr() {
    tx_busy_ = false;
}

void Communication::processErrorFromIsr() {
    if ((uart_ == nullptr) || (!initialized_)) {
        return;
    }

    (void)HAL_UART_AbortReceive(uart_);
    dma_read_position_ = 0U;
    parser_.reset();

    SCB_CleanInvalidateDCache_by_Addr(
        reinterpret_cast<uint32_t*>(dma_rx_buffer_),
        sizeof(dma_rx_buffer_)
    );

    (void)HAL_UARTEx_ReceiveToIdle_DMA(
        uart_,
        dma_rx_buffer_,
        sizeof(dma_rx_buffer_)
    );
}

} // namespace communication

extern "C" void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef* huart,
    uint16_t size
) {
    if (huart == &huart10) {
        communication::g_communication.processRxEventFromIsr(size);
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart10) {
        communication::g_communication.processTxCompleteFromIsr();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart10) {
        communication::g_communication.processErrorFromIsr();
    }
}
