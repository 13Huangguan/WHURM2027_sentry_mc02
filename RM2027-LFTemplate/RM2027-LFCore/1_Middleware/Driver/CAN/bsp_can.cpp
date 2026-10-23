/** @file bsp_can.cpp @brief 三路 FDCAN 收发、接收观察者与故障诊断。 */
#include "bsp_can.h"
#include "fdcan.h"
#include "main.h"

namespace {
class IrqLock {
public:
    IrqLock() : mask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~IrqLock() { __DMB(); __set_PRIMASK(mask_); }
private:
    uint32_t mask_;
};
BspFdcanStats stats[3] = {};
BspFdcanObserver observers[8] = {};
uint8_t observer_count = 0;
constexpr uint32_t kErrorNotifications = FDCAN_IT_BUS_OFF
    | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE;

// 跟随 CubeMX 分配：有 FIFO0 时优先使用，否则使用 FIFO1。
uint32_t receiveFifo(const FDCAN_HandleTypeDef* bus)
{
    return bus->Init.RxFifo0ElmtsNbr != 0U ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
}
int busIndex(FDCAN_HandleTypeDef* bus)
{
    if (bus == &hfdcan1) { return 0; }
    if (bus == &hfdcan2) { return 1; }
    if (bus == &hfdcan3) { return 2; }
    return -1;
}
bool startBus(FDCAN_HandleTypeDef* bus)
{
    if (bus->Init.RxFifo0ElmtsNbr == 0U && bus->Init.RxFifo1ElmtsNbr == 0U) {
        return false;
    }
    const bool fifo0 = receiveFifo(bus) == FDCAN_RX_FIFO0;
    const uint32_t notifications = kErrorNotifications
        | (fifo0 ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST
                 : FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST);
    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = fifo0 ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    return HAL_FDCAN_ConfigFilter(bus, &filter) == HAL_OK
        && HAL_FDCAN_ConfigGlobalFilter(bus, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK
        && HAL_FDCAN_Start(bus) == HAL_OK
        && HAL_FDCAN_ActivateNotification(bus, notifications, 0) == HAL_OK;
}
}
void bspFdcanInit(void)
{
    bspFdcanInitBus(&hfdcan1);
    bspFdcanInitBus(&hfdcan2);
    bspFdcanInitBus(&hfdcan3);
}
void bspFdcanInitBus(FDCAN_HandleTypeDef* bus)
{
    if (busIndex(bus) < 0) { return; }
    if (!startBus(bus)) { Error_Handler(); }
}
FDCAN_HandleTypeDef* bspFdcanGetHandle(uint8_t index)
{
    switch (index) {
        case 1: return &hfdcan1;
        case 2: return &hfdcan2;
        case 3: return &hfdcan3;
        default: return nullptr;
    }
}
bool bspFdcanSend(FDCAN_HandleTypeDef* bus, uint32_t identifier,
                  const uint8_t* data, uint8_t length)
{
    const int index = busIndex(bus);
    if (index < 0) { return false; }
    // FIFO 索引检查和写入保持原子性，不在临界区轮询等待。
    IrqLock lock;
    auto& status = stats[index];
    if (identifier > 0x7FFU || length > 8U || (length != 0U && data == nullptr)
        || status.bus_off || status.recovering
        || HAL_FDCAN_GetTxFifoFreeLevel(bus) == 0U) {
        ++status.tx_failed;
        return false;
    }
    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = length; // 当前 HAL 的 DLC 0~8 就是字节数。
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // HAL 按 4 字节块读取 TX 数据，短帧也使用完整缓冲。
    uint8_t payload[8] = {};
    for (uint8_t i = 0; i < length; ++i) { payload[i] = data[i]; }
    if (HAL_FDCAN_AddMessageToTxFifoQ(bus, &header, payload) != HAL_OK) {
        ++status.tx_failed;
        status.last_hal_error = bus->ErrorCode;
        return false;
    }
    ++status.tx_queued;
    return true;
}
bool bspFdcanAddObserver(BspFdcanObserver observer)
{
    IrqLock lock;
    if (observer == nullptr || observer_count >= 8U) { return false; }
    for (uint8_t i = 0; i < observer_count; ++i) {
        if (observers[i] == observer) { return false; }
    }
    observers[observer_count++] = observer;
    return true;
}
bool bspFdcanGetStats(FDCAN_HandleTypeDef* bus, BspFdcanStats* result)
{
    const int index = busIndex(bus);
    if (index < 0 || result == nullptr) { return false; }
    IrqLock lock;
    *result = stats[index];
    return true;
}
bool bspFdcanRecover(FDCAN_HandleTypeDef* bus)
{
    const int index = busIndex(bus);
    if (index < 0 || __get_IPSR() != 0U || __get_PRIMASK() != 0U) { return false; }
    {
        IrqLock lock;
        if (stats[index].recovering) { return false; }
        stats[index].recovering = true;
    }
    // 仅任务上下文调用，调用方停止其他发送者。清空旧 TX，避免恢复后重放。
    const bool ok = HAL_FDCAN_DeInit(bus) == HAL_OK && HAL_FDCAN_Init(bus) == HAL_OK
                    && startBus(bus);
    {
        IrqLock lock;
        stats[index].recovering = false;
        stats[index].bus_off = !ok;
        if (ok) { ++stats[index].recoveries; }
        else { ++stats[index].errors; stats[index].last_hal_error = bus->ErrorCode; }
    }
    return ok;
}
static void receiveFrames(FDCAN_HandleTypeDef* bus, uint32_t fifo, uint32_t flags)
{
    const int index = busIndex(bus);
    if (index < 0) { return; }
    const bool fifo0 = fifo == FDCAN_RX_FIFO0;
    const uint32_t lost = fifo0 ? FDCAN_IT_RX_FIFO0_MESSAGE_LOST : FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    const uint32_t message = fifo0 ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    const uint32_t capacity = fifo0 ? bus->Init.RxFifo0ElmtsNbr : bus->Init.RxFifo1ElmtsNbr;
    if ((flags & lost) != 0U) { ++stats[index].rx_lost; }
    if ((flags & (message | lost)) == 0U) {
        return;
    }
    static constexpr uint8_t kDlcBytes[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    for (uint32_t i = 0; i < capacity
         && HAL_FDCAN_GetRxFifoFillLevel(bus, fifo) != 0U; ++i) {
        FDCAN_RxHeaderTypeDef header{};
        uint8_t data[64] = {};
        if (HAL_FDCAN_GetRxMessage(bus, fifo, &header, data) != HAL_OK) {
            ++stats[index].errors;
            stats[index].last_hal_error = bus->ErrorCode;
            break;
        }
        ++stats[index].rx_frames;
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME
            || header.DataLength > 15U) { continue; }
        const uint8_t length = kDlcBytes[header.DataLength];
        if (header.FDFormat == FDCAN_CLASSIC_CAN && length > 8U) { continue; }
        if (header.FDFormat == FDCAN_CLASSIC_CAN && length <= 8U) {
            bspFdcanRxHandler(bus, header.Identifier, data, length);
        }
        for (uint8_t observer = 0; observer < observer_count; ++observer) {
            observers[observer](bus, &header, data, length);
        }
    }
}
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* bus, uint32_t flags)
{
    receiveFrames(bus, FDCAN_RX_FIFO0, flags);
}
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* bus, uint32_t flags)
{
    receiveFrames(bus, FDCAN_RX_FIFO1, flags);
}
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* bus, uint32_t flags)
{
    const int index = busIndex(bus);
    if (index < 0) { return; }
    ++stats[index].errors;
    stats[index].last_hal_error = bus->ErrorCode;
    if ((flags & FDCAN_IT_BUS_OFF) != 0U && (bus->Instance->PSR & FDCAN_PSR_BO) != 0U) {
        ++stats[index].bus_off_count;
        stats[index].bus_off = true;
    }
}
extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* bus)
{
    const int index = busIndex(bus);
    if (index >= 0) { ++stats[index].errors; stats[index].last_hal_error = bus->ErrorCode; }
}
__attribute__((weak)) void bspFdcanRxHandler(FDCAN_HandleTypeDef*, uint32_t,
                                            const uint8_t*, uint8_t) {}
