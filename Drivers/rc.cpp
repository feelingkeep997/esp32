// ============================================================================
// rc.cpp —— 遥控接收机（SBUS / CRSF，串口）
//
// 职责:
//   从 UART 读取遥控接收机的串口数据，解析出各通道原始值，并归一化成飞控使用的
//   controlRoll/Pitch/Yaw/Throttle/Mode（[-1,1] / [0,1]），同时刷新 controlTime
//   供"遥控丢失保护"判断。
//
// 关键逻辑:
//   - setupRC() 按 rcBaud 安装并配置 UART（默认 CRSF 420000bps）；
//   - readRC() 每个控制周期被调用，逐字节喂入协议状态机：
//       rcProtocol == 0：SBUS（0x0F 起始 + 16×11bit 通道 + 结束标志）；
//       rcProtocol == 1：CRSF（当前仅为占位实现，见下）；
//   - 一帧解析完成后调用 normalizeRC()，把原始通道值按 channelZero/channelMax 映射成归一值，
//     并把 controlTime 置为当前时间。
//
// 输入/输出:
//   输入：UART（引脚 BOARD_RC_RX_PIN / rcRxPin）。
//   输出：channels[16]、controlRoll/Pitch/Yaw/Throttle/Mode、controlTime。
//
// 重要参数:
//   rcProtocol(0=SBUS,1=CRSF) / rcBaud(420000) / rcTxPin / rcRxPin
//   channelZero[16] / channelMax[16] 通道标定值（默认 999/2099，可由参数系统改写）
//
// 边界情况与潜在风险:
//   - CRSF 分支是**未完成的占位实现**（TODO）：只收字节不清包，不会更新任何通道，
//     因此当 rcProtocol=1 时遥控实际不工作——这是当前工程最需要注意的功能缺口。
//   - SBUS 分支的 11bit 解包写法可疑：`temp = byte & 0x7FF`（byte 为 uint8_t，掩码无实际作用）
//     与 `temp |= (byte & 0x7F) << 11`（标准 SBUS 应为 << 8 且取低 3 位）疑似有误，
//     会导致通道值错误；如需启用 SBUS 请重点核对这段。
//   - normalizeRC() 用 mapf 做线性映射且不做钳位，超范围输入会得到超范围输出。
//   - channelZero/channelMax 默认值为占位值，实际使用前必须标定（calibrateRC 目前也是空实现）。
//   - controlTime 只在成功解析一帧后更新；若接收机无信号，它保持不变，
//     从而使 rcLossFailsafe 能按 t - controlTime 判断超时。
//   - 逐字节非阻塞读（超时 0），单次调用最多消费一个字节，需保证调用频率足够高。
// ============================================================================

#include "globals.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "rc";

int rcProtocol = 1; // 0=SBUS, 1=CRSF
int rcBaud = 420000;
int rcTxPin = -1;
int rcRxPin = BOARD_RC_RX_PIN;
uint16_t channels[16];
float controlTime = 0;
float channelZero[16] = {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999};
float channelMax[16] = {2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099, 2099};
float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
static uart_port_t rcUartPort = UART_NUM_1;
static bool rcInitialized = false;

void setupRC() {
    print("Setting up RC receiver...\n");
    
    // Configure UART for RC protocol
    uart_config_t uart_config = {};
    uart_config.baud_rate = rcBaud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    
    esp_err_t ret = uart_driver_install(rcUartPort, 256, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        print("Failed to install RC UART driver\n");
        return;
    }
    
    ret = uart_param_config(rcUartPort, &uart_config);
    if (ret != ESP_OK) {
        print("Failed to configure RC UART params\n");
        return;
    }
    
    ret = uart_set_pin(rcUartPort, rcTxPin, rcRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        print("Failed to set RC UART pins\n");
        return;
    }
    
    rcInitialized = true;
    print("RC receiver initialized at %d baud\n", rcBaud);
}

// 从 UART 读取并解析遥控数据。返回 true 表示本次成功解析出完整一帧。
// 非阻塞：每次最多消费 1 个字节，由主循环高频调用逐步喂入状态机。
// 注意：CRSF 分支为占位实现，不会返回 true（详见文件头风险）。
bool readRC() {
    if (!rcInitialized) return false;
    
    uint8_t byte;
    static int idx = 0;              // SBUS 状态机进度
    static uint8_t buffer[22];       // SBUS 帧缓冲
    
    // 非阻塞读取 1 字节（超时 0）
    int bytesRead = uart_read_bytes(rcUartPort, &byte, 1, pdMS_TO_TICKS(0));
    
    if (bytesRead > 0 && rcProtocol == 0) { // SBUS
        // SBUS：帧周期 17~25ms，0x0F 起始、16 个 11bit 通道、0x0F 结束标志
        if (idx == 0 && byte == 0x0F) { // 起始标志
            idx = 1;
            buffer[0] = byte;
        } else if (idx > 0 && idx < 21) {
            // SBUS 的 11bit 通道按小端方式跨字节打包（此处实现见文件头风险提示）
            static bool lsb = true;
            static uint16_t temp;
            
            if (lsb) {
                temp = byte & 0x7FF;
                lsb = false;
            } else {
                temp |= ((byte & 0x7F) << 11);
                channels[idx - 1] = temp;
                idx++;
                lsb = true;
            }
        } else if (idx == 21) {
            // 结束标志校验
            if (byte == 0x0F) {
                normalizeRC();
                idx = 0;
                return true;
            } else {
                idx = 0; // 帧尾异常，丢弃重同步
            }
        }
    } else if (bytesRead > 0 && rcProtocol == 1) { // CRSF
        // CRSF：串口包流。TODO: 尚未实现真正的解析
        static uint8_t crsfBuffer[64];
        static int crsfIdx = 0;
        
        if (crsfIdx < sizeof(crsfBuffer)) {
            crsfBuffer[crsfIdx++] = byte;
            
            // 简化启发式：攒够一定字节就当作一包处理（实际未解析内容）
            if (crsfIdx >= 10) {
                // TODO: 实现完整的 CRSF 解析
                crsfIdx = 0;
            }
        }
    }
    
    return false;
}

// 把 channels[] 原始值按标定值线性映射成归一化控制量，并刷新 controlTime。
// 映射：roll/pitch/yaw → [-1,1]；throttle → [0,1]；mode → [0,2]（对应三档模式）。
// 注意：不做钳位，超出标定范围的输入会产生超出 [0,1]/[-1,1] 的输出。
void normalizeRC() {
    // 通道映射：0=roll, 1=pitch, 2=throttle, 3=yaw, 4=mode
    rollChannel = mapf(channels[0], channelZero[0], channelMax[0], -1, 1);
    pitchChannel = mapf(channels[1], channelZero[1], channelMax[1], -1, 1);
    throttleChannel = mapf(channels[2], channelZero[2], channelMax[2], 0, 1);
    yawChannel = mapf(channels[3], channelZero[3], channelMax[3], -1, 1);
    modeChannel = mapf(channels[4], channelZero[4], channelMax[4], 0, 2);
    
    controlTime = t; // 记录最近一次收到遥控数据的时间（供丢失保护判断）
}

void calibrateRC() {
    print("Calibrating RC...\n");
    print("Move sticks to extremes and hold for 2 seconds each\n");
    
    // Placeholder for RC calibration
    // TODO: Implement proper calibration routine
}

void calibrateRCChannel(float *channel, uint16_t in[16], uint16_t out[16], const char *name) {
    // Placeholder
}

void printRCCalibration() {
    print("RC Calibration:\n");
    print("CH0 zero=%u max=%u\n", (unsigned)channelZero[0], (unsigned)channelMax[0]);
    print("CH1 zero=%u max=%u\n", (unsigned)channelZero[1], (unsigned)channelMax[1]);
    print("CH2 zero=%u max=%u\n", (unsigned)channelZero[2], (unsigned)channelMax[2]);
    print("CH3 zero=%u max=%u\n", (unsigned)channelZero[3], (unsigned)channelMax[3]);
    print("CH4 zero=%u max=%u\n", (unsigned)channelZero[4], (unsigned)channelMax[4]);
}
