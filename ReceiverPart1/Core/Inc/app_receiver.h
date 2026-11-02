#ifndef APP_RECEIVER_H
#define APP_RECEIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CH1/CH2 数据帧协议（所有多字节字段均为小端）：
 *
 * 偏移       长度   含义
 * 0..1       2      同步头 A5 5A
 * 2          1      协议版本，目前固定为 01
 * 3          1      10=CH1，20=CH2，30=CH1+CH2
 * 4..5       2      帧序号
 * 6..7       2      每通道采样点数，范围 1~500
 * 8..9       2      相邻采样点间隔，单位 us
 * 10         1      通道标志，bit0=CH1，bit1=CH2
 * 11         1      保留，必须写 0
 * 12...      变长   电压值，单位 mV，0~3000
 * 最后 2 字节        对前面全部字节做 CRC-16/CCITT-FALSE
 *
 * 单独 CH1/CH2 帧的数据排列：
 *   12.. : uint16_t sample[0], sample[1], ...
 *
 * 双通道帧 0x30 的交错排列：
 *   12.. : CH1[0], CH2[0], D[0], CH1[1], CH2[1], D[1], ...
 *
 * 扩展数字帧的通道标志为 0x07。每个采样点占 5 字节：
 *   CH1 低字节、CH1 高字节、CH2 低字节、CH2 高字节、数字位图。
 *   数字位图 bit0/bit1/bit2 分别对应数字通道 D1/D2/D3。
 *
 * 任务通道含义：
 *   CH1 = 电位器，ADC3_IN7 / PF9 / Arduino A2；
 *   CH2 = 麦克风，ADC3_IN0 / PA0 / Arduino A0。
 */

/* 初始化 SDRAM、LCD、串口接收和初始界面，只调用一次。 */
void App_ReceiverInit(void);

/* 在主循环中反复调用，负责解析字节、触摸换挡、PI11 暂停和刷新屏幕。 */
void App_ReceiverLoop(void);

/*
 * 接收端 ACK 为文本：
 * ACK,<status>,<sequence>,<points>,<interval_us>,<ok>,<crc>,<drops>\r\n
 * status: OK / CRC / FORMAT。
 */

/* 一帧中每个通道最多 500 点。 */
#define APP_FRAME_MAX_SAMPLES 500U

/* 固定帧头长度为 12 字节。 */
#define APP_FRAME_HEADER_SIZE 12U

/* 双通道模拟量加三路数字位图：12 + 5*500 + 2 = 2514 字节。 */
#define APP_FRAME_MAX_SIZE \
  (APP_FRAME_HEADER_SIZE + (5U * APP_FRAME_MAX_SAMPLES) + 2U)

/* 以下统计接口适合在 CubeIDE 的 Expressions 窗口或断点中观察。 */
uint32_t App_GetValidFrameCount(void);
uint32_t App_GetCrcErrorCount(void);
uint32_t App_GetRingDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RECEIVER_H */
