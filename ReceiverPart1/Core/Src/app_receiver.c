/*
 * 接收端应用层代码说明：
 * CubeMX 生成 GPIO、USART1、FMC 和 LTDC 的基础配置；
 * 本文件负责数据包接收、CRC 校验、触摸/按键换档和双通道波形绘制。
 */

#include "app_receiver.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "board_link.h"
#include "fmc.h"
#include "ltdc.h"
#include "main.h"
#include "touchscreen.h"
#include "usart.h"

/*
 * ============================ 应用层总览 ============================
 * 1. UART7/板间数据和 USART1/PC 测试字节分别写入环形缓冲区；
 * 2. 主循环从环形缓冲区取字节，寻找 A5 5A 帧头；
 * 3. 根据帧类型解析 CH1、CH2 或 CH1+CH2 数据；
 * 4. 对整帧执行 CRC-16 校验；
 * 5. 把 0~3000 mV 转换为屏幕 Y 坐标；
 * 6. 先画到 SDRAM 后台缓冲区，再切换 LTDC 地址，避免闪烁；
 * 7. 触摸屏按钮切换扫描档位，板载 PI11 按键暂停/继续波形；
 * 8. 每帧返回带采样间隔和接收统计的文本 ACK。
 *
 * 文件中的“帧”表示一次完整波形数据包，不是动画的一帧。
 * ===================================================================
 */

/* ============================ 1. 屏幕与绘图区域 ============================ */
#define LCD_WIDTH 480U
#define LCD_HEIGHT 272U
#define LCD_FRAME_PIXELS (LCD_WIDTH * LCD_HEIGHT)
#define LCD_FRAME_STRIDE 0x00040000UL
/* 两个 SDRAM 帧缓冲区：显示一帧时，CPU 绘制另一帧，避免屏幕撕裂。 */
#define LCD_FB0_ADDRESS 0xC0000000UL
#define LCD_FB1_ADDRESS (LCD_FB0_ADDRESS + LCD_FRAME_STRIDE)
#define LCD_BACKGROUND_ADDRESS (LCD_FB1_ADDRESS + LCD_FRAME_STRIDE)
#define LCD_SNAPSHOT_ADDRESS (LCD_BACKGROUND_ADDRESS + LCD_FRAME_STRIDE)

#define PLOT_LEFT 42
#define PLOT_RIGHT 474
#define PLOT_TOP 78
#define ANALOG_PLOT_BOTTOM 168
#define DIGITAL_AREA_TOP 176
#define PLOT_BOTTOM 236
#define PLOT_WIDTH ((PLOT_RIGHT - PLOT_LEFT) + 1)
#define PLOT_HEIGHT ((ANALOG_PLOT_BOTTOM - PLOT_TOP) + 1)

/* ============================ 2. 上下行协议 ============================ */
#define FRAME_SYNC_0 0xA5U
#define FRAME_SYNC_1 0x5AU
#define FRAME_VERSION 0x01U
#define FRAME_TYPE_CH1 0x10U
#define FRAME_TYPE_CH2 0x20U
#define FRAME_TYPE_CH1_CH2 0x30U
#define FRAME_FLAG_CH1 0x01U
#define FRAME_FLAG_CH2 0x02U
#define FRAME_FLAG_DIGITAL 0x04U
#define FRAME_FLAG_CH1_CH2 (FRAME_FLAG_CH1 | FRAME_FLAG_CH2)
#define FRAME_FLAG_ALL (FRAME_FLAG_CH1_CH2 | FRAME_FLAG_DIGITAL)

#define RX_BYTES_PER_LOOP 4096U
#define BUTTON_DEBOUNCE_MS 35U
#define TOUCH_DEBOUNCE_MS 120U
#define TOUCH_COMMAND_RETRY_MS 150U
#define TOUCH_COMMAND_RETRIES 3U
#define PC_COMMAND_MAX_LENGTH 15U
#define SAVE_KEY_DEBOUNCE_MS 35U

#define CONTROL_SYNC_0 0xC5U
#define CONTROL_SYNC_1 0x5CU
#define CONTROL_VERSION 0x01U
#define CONTROL_CMD_TIMEBASE 0x01U
#define CONTROL_CMD_FREQUENCY 0x02U

/* 顶部五个直接档位按钮。 */
#define TIMEBASE_COUNT 5U
#define TOUCH_BUTTON_Y0 4
#define TOUCH_BUTTON_Y1 35
#define TOUCH_BUTTON_X0 266
#define TOUCH_BUTTON_WIDTH 42
#define TOUCH_BUTTON_GAP 1

#define CHANNEL_BUTTON_Y0 42
#define CHANNEL_BUTTON_Y1 68
#define CHANNEL_BUTTON_WIDTH 54
#define CHANNEL_BUTTON_GAP 6
#define CHANNEL_BUTTON_X0 8
#define CHANNEL_BUTTON_COUNT 5U

#define DIGITAL_CHANNEL_COUNT 3U
#define DIGITAL_LANE_HEIGHT 18
#define DIGITAL_SCROLL_STEP 2U

#define FREQ_SLIDER_X0 312
#define FREQ_SLIDER_X1 470
#define FREQ_TRACK_X0 330
#define FREQ_TRACK_X1 468
#define FREQ_SLIDER_Y0 42
#define FREQ_SLIDER_Y1 68
#define FREQ_CONTROL_MIN 100U
#define FREQ_CONTROL_MAX 10000U
#define FREQ_SWITCH_TO_5MS_ABOVE 100U
#define FREQ_SWITCH_TO_1MS_ABOVE 500U
#define FREQ_SWITCH_TO_500US_ABOVE 2000U
#define FREQ_SWITCH_TO_100US_ABOVE 8000U
#define ANALOG_FREQUENCY_DIVISOR 2U
#define CH2_DISPLAY_POINT_DIVISOR 2U

/* 跨功能段提前声明，具体定义紧挨着对应功能。 */
static void select_timebase(uint8_t index);
static bool capture_snapshot(void);
static void send_csv_snapshot(void);
static void send_bmp_snapshot(void);
static void refresh_timebase_screen(void);

/* 5×7 点阵字库：只保存数字和英文大写字母，节省 Flash。 */
static const uint8_t font_5x7[36][5] =
{
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
  {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
  {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
  {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
  {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
  {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
  {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
  {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
  {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 9 */
  {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
  {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
  {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
  {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
  {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
  {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
  {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
  {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
  {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
  {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
  {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
  {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
  {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
  {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
  {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
  {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
  {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
  {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
  {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
  {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
  {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
  {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
  {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
  {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
  {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
  {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z */
};

static uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
  /* 把 8-8-8 位颜色压成 LCD 使用的 5-6-5 位格式。 */
  return (uint16_t)(((uint16_t)(red & 0xF8U) << 8) | // 红色放高 5 位
                    ((uint16_t)(green & 0xFCU) << 3) | // 绿色放中间 6 位
                    ((uint16_t)blue >> 3));             // 蓝色放低 5 位
}

#define COLOR_BACKGROUND rgb565(2, 8, 18)
#define COLOR_PANEL rgb565(7, 22, 38)
#define COLOR_PLOT rgb565(0, 4, 12)
#define COLOR_GRID rgb565(22, 48, 68)
#define COLOR_GRID_MAJOR rgb565(48, 86, 112)
#define COLOR_TEXT rgb565(220, 238, 246)
#define COLOR_MUTED rgb565(132, 162, 184)
/* CH1 uses bright cyan and CH2 amber, so neither is confused with grid lines. */
#define COLOR_CH1 rgb565(32, 220, 255)
#define COLOR_CH2 rgb565(255, 176, 32)
#define COLOR_OK rgb565(64, 224, 112)
#define COLOR_WARN rgb565(255, 178, 48)
#define COLOR_ERROR rgb565(255, 72, 72)
#define COLOR_DIGITAL1 rgb565(80, 255, 128)
#define COLOR_DIGITAL2 rgb565(255, 108, 200)
#define COLOR_DIGITAL3 rgb565(150, 170, 255)

/* Keep the task's physical meaning visible next to each trace. */
#define CHANNEL_LABEL_CH1 "CH1 POT"
#define CHANNEL_LABEL_CH2 "CH2 MIC"
#define CHANNEL_VALUE_OFFSET 48

/* ============================ 4. 基础绘图函数 ============================ */
/*
 * 在指定帧缓冲区写一个像素。
 * framebuffer 是显示器内存的起始地址，x/y 为屏幕坐标。
 * 越界时直接忽略，防止数组写入非法内存。
 */
static void pixel_put(uint16_t *framebuffer, int x, int y, uint16_t color)
{
  if ((x >= 0) && (x < (int)LCD_WIDTH) &&
      (y >= 0) && (y < (int)LCD_HEIGHT)) // 坐标必须先做边界检查
  {
    framebuffer[((uint32_t)y * LCD_WIDTH) + (uint32_t)x] = color; // 行优先寻址
  }
}

/* 用同一种颜色填充矩形区域，x0/y0 为左上角，x1/y1 为右下角。 */
static void fill_rect(uint16_t *framebuffer, int x0, int y0, int x1, int y1,
                      uint16_t color)
{
  int x;
  int y;

  if (x0 < 0) x0 = 0;                                      // 左边界裁剪
  if (y0 < 0) y0 = 0;                                      // 上边界裁剪
  if (x1 >= (int)LCD_WIDTH) x1 = (int)LCD_WIDTH - 1;       // 右边界裁剪
  if (y1 >= (int)LCD_HEIGHT) y1 = (int)LCD_HEIGHT - 1;     // 下边界裁剪
  if ((x0 > x1) || (y0 > y1)) return;                      // 裁剪后无有效区域

  for (y = y0; y <= y1; ++y)
  {
    uint16_t *row = framebuffer + ((uint32_t)y * LCD_WIDTH); // 当前行首地址
    for (x = x0; x <= x1; ++x)
    {
      row[x] = color; // 连续写入同一行像素
    }
  }
}

/* 画一条水平线，内部复用矩形填充。 */
static void line_h(uint16_t *framebuffer, int x0, int x1, int y,
                   uint16_t color)
{
  if (x0 > x1) // 统一成从左向右绘制
  {
    int temp = x0;
    x0 = x1;
    x1 = temp;
  }
  fill_rect(framebuffer, x0, y, x1, y, color); // 高度为 1 的矩形
}

/* 画一条垂直线，内部复用矩形填充。 */
static void line_v(uint16_t *framebuffer, int x, int y0, int y1,
                   uint16_t color)
{
  if (y0 > y1) // 统一成从上向下绘制
  {
    int temp = y0;
    y0 = y1;
    y1 = temp;
  }
  fill_rect(framebuffer, x, y0, x, y1, color); // 宽度为 1 的矩形
}

/* 画任意方向的连续线段，避免稀疏采样点之间出现一格一格的断点。 */
static void line_segment(uint16_t *framebuffer, int x0, int y0,
                         int x1, int y1, uint16_t color)
{
  int dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1); // X 方向绝对距离
  int sx = (x0 < x1) ? 1 : -1;                 // X 方向步进符号
  int dy = (y1 >= y0) ? (y0 - y1) : (y1 - y0); // Y 方向负绝对距离
  int sy = (y0 < y1) ? 1 : -1;                 // Y 方向步进符号
  int error = dx + dy;                         // Bresenham 误差初值

  for (;;)
  {
    pixel_put(framebuffer, x0, y0, color); // 绘制当前线段像素
    if ((x0 == x1) && (y0 == y1))
    {
      break; // 已到终点
    }
    int error2 = 2 * error; // 比较下一步是否需要移动
    if (error2 >= dy)
    {
      error += dy;
      x0 += sx; // 向终点靠近一列
    }
    if (error2 <= dx)
    {
      error += dx;
      y0 += sy; // 向终点靠近一行
    }
  }
}

/*
 * 返回一个字符的 5×7 点阵数据。
 * 每个字符由 5 列组成，每列 7 位有效，位值为 1 的位置会点亮。
 */
static const uint8_t *glyph_for(char character)
{
  static const uint8_t space[5] = {0, 0, 0, 0, 0};              // 空白
  static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00}; // 小数点
  static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00}; // 冒号
  static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08}; // 负号
  static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02}; // 斜杠

  if ((character >= '0') && (character <= '9'))
  {
    return font_5x7[character - '0']; // 数字 0~9
  }
  if ((character >= 'A') && (character <= 'Z'))
  {
    return font_5x7[10 + (character - 'A')]; // 字母 A~Z
  }
  if (character == '.') return dot;
  if (character == ':') return colon;
  if (character == '-') return dash;
  if (character == '/') return slash;
  return space; // 不支持的字符显示为空格
}

/*
 * 在 framebuffer 上绘制字符串。
 * scale=1 为正常大小，scale=2 时宽高都放大两倍。
 * 返回值是字符串末尾的 x 坐标，便于继续拼接下一段文字。
 */
static int draw_text(uint16_t *framebuffer, int x, int y, const char *text,
                     uint16_t color, uint8_t scale)
{
  int cursor = x; // 当前字符左侧起点

  while (*text != '\0')
  {
    const uint8_t *glyph = glyph_for(*text++); // 取出当前字符点阵
    uint8_t column;
    uint8_t row;

    for (column = 0; column < 5U; ++column)
    {
      for (row = 0; row < 7U; ++row)
      {
        if (((glyph[column] >> row) & 0x01U) != 0U) // 该点需要点亮
        {
          fill_rect(framebuffer, // scale 大于 1 时画成放大方块
                    cursor + ((int)column * scale),
                    y + ((int)row * scale),
                    cursor + ((int)column * scale) + scale - 1,
                    y + ((int)row * scale) + scale - 1,
                    color);
        }
      }
    }
    cursor += 6 * scale; // 5 列点阵加 1 列字符间距
  }
  return cursor; // 返回下一字符起点
}

/* 把整数毫伏值格式化成 0.000V~3.000V 的屏幕字符串。 */
static void format_voltage(uint16_t millivolts, char *output)
{
  uint16_t whole = millivolts / 1000U;    // 伏特整数部分
  uint16_t fraction = millivolts % 1000U; // 毫伏小数部分

  output[0] = (char)('0' + whole);                        // 个位伏特
  output[1] = '.';                                        // 小数点
  output[2] = (char)('0' + (fraction / 100U));            // 十分位
  output[3] = (char)('0' + ((fraction / 10U) % 10U));     // 百分位
  output[4] = (char)('0' + (fraction % 10U));             // 千分位
  output[5] = 'V';                                        // 电压单位
  output[6] = '\0';                                       // 字符串结束符
}

/* 将无符号整数转换为十进制字符串，不依赖 printf，减少代码体积。 */
static void format_uint(char *output, uint32_t value)
{
  char reverse[11]; // uint32_t 最多 10 位十进制数字
  uint8_t count = 0; // 已生成数字位数
  uint8_t index = 0; // 输出位置

  if (value == 0U)
  {
    output[0] = '0'; // 特殊处理零
    output[1] = '\0';
    return;
  }

  while ((value > 0U) && (count < sizeof(reverse)))
  {
    reverse[count++] = (char)('0' + (value % 10U)); // 取最低位
    value /= 10U;                                   // 去掉最低位
  }
  while (count > 0U)
  {
    output[index++] = reverse[--count]; // 反向写回得到正序
  }
  output[index] = '\0'; // 添加字符串结束符
}

/* ============================ 5. 屏幕布局与波形渲染 ============================ */

/* 扫描档位状态：与按钮绘制和换档函数放在一起。 */
static const uint16_t timebase_interval_us[TIMEBASE_COUNT] =
    {200U, 100U, 20U, 10U, 2U}; // 每点采样间隔
static const char *const timebase_labels[TIMEBASE_COUNT] =
    {"10MS", "5MS", "1MS", "500US", "100US"}; // 屏幕按钮文字
static uint8_t timebase_index;              // 当前显示档位
static bool timebase_pending;               // 是否正在等待采集端确认
static uint8_t timebase_pending_index;      // 目标档位
static uint8_t timebase_retries_left;       // 剩余重发次数
static uint32_t timebase_retry_at_ms;       // 下一次重发时间
static bool timebase_failed;                // 换档是否失败
static uint32_t timebase_failed_at_ms;      // 失败时刻
static bool timebase_user_selected;         // 是否正在等待用户选择生效

/** 绘制五个扫描时间按钮，并高亮当前档位。 */
static void draw_timebase_buttons(uint16_t *framebuffer)
{
  uint8_t index;

  for (index = 0U; index < TIMEBASE_COUNT; ++index)
  {
    int x0 = TOUCH_BUTTON_X0 + // 当前按钮左边界
             ((int)index * (TOUCH_BUTTON_WIDTH + TOUCH_BUTTON_GAP));
    int x1 = x0 + TOUCH_BUTTON_WIDTH - 1; // 当前按钮右边界
    int text_x =
        x0 + ((TOUCH_BUTTON_WIDTH - ((int)strlen(timebase_labels[index]) * 6)) / 2);
    uint16_t fill = (index == timebase_index) ? COLOR_CH1 : COLOR_PANEL; // 选中高亮
    uint16_t text = (index == timebase_index) ? COLOR_BACKGROUND : COLOR_TEXT; // 文字反色

    fill_rect(framebuffer, x0, TOUCH_BUTTON_Y0, x1, TOUCH_BUTTON_Y1, fill); // 按钮底色
    draw_text(framebuffer, text_x, 15, timebase_labels[index], text, 1);     // 档位文字
  }
}

/* 通道按钮文字和可见状态，供按钮绘制、触摸切换和波形渲染共同使用。 */
static const char *const channel_labels[CHANNEL_BUTTON_COUNT] =
{
  "A1", "A2", "D1", "D2", "D3"
};
static bool channel_visible[CHANNEL_BUTTON_COUNT]; // 各通道是否显示

static uint16_t channel_color(uint8_t index)
{
  switch (index)
  {
    case 0U: return COLOR_CH1;      // CH1
    case 1U: return COLOR_CH2;      // CH2
    case 2U: return COLOR_DIGITAL1; // D1
    case 3U: return COLOR_DIGITAL2; // D2
    default: return COLOR_DIGITAL3; // D3
  }
}

/** 计算指定通道按钮的横坐标。 */
static int channel_button_x(uint8_t index)
{
  return CHANNEL_BUTTON_X0 +
         ((int)index * (CHANNEL_BUTTON_WIDTH + CHANNEL_BUTTON_GAP));
}

/** 绘制五个通道开关，按钮颜色表示通道是否开启。 */
static void draw_channel_buttons(uint16_t *framebuffer)
{
  for (uint8_t index = 0U; index < CHANNEL_BUTTON_COUNT; ++index)
  {
    int x0 = channel_button_x(index);                 // 按钮左边界
    int x1 = x0 + CHANNEL_BUTTON_WIDTH - 1;          // 按钮右边界
    uint16_t color = channel_color(index);            // 对应通道颜色

    fill_rect(framebuffer, x0, CHANNEL_BUTTON_Y0, x1, CHANNEL_BUTTON_Y1,
              channel_visible[index] ? color : COLOR_PANEL); // 关闭时变暗
    draw_text(framebuffer, x0 + 18, CHANNEL_BUTTON_Y0 + 8,
              channel_labels[index],
              channel_visible[index] ? COLOR_BACKGROUND : COLOR_MUTED, 1); // 文字颜色
  }
}

/** 把频率滑块的触摸位置换算成 100Hz~10kHz 频率。 */
static uint16_t frequency_from_slider(int x)
{
  int span = FREQ_TRACK_X1 - FREQ_TRACK_X0; // 轨道总宽度
  int half = span / 2;                      // 1kHz 中点位置
  int position;

  if (x <= FREQ_TRACK_X0) return FREQ_CONTROL_MIN;
  if (x >= FREQ_TRACK_X1) return FREQ_CONTROL_MAX;
  position = x - FREQ_TRACK_X0;
  if (position <= half)
  {
    return (uint16_t)(FREQ_CONTROL_MIN + // 左半段 100Hz~1kHz
                      ((uint32_t)position * 900U) / (uint32_t)half);
  }
  return (uint16_t)(1000U + // 右半段 1kHz~10kHz
                    ((uint32_t)(position - half) * 9000U) /
                        (uint32_t)(span - half));
}

/** 把当前频率换算成滑块旋钮的横坐标。 */
static int slider_x_from_frequency(uint16_t frequency)
{
  int span = FREQ_TRACK_X1 - FREQ_TRACK_X0;
  int half = span / 2;

  if (frequency <= 1000U)
  {
    return FREQ_TRACK_X0 +
           ((int)(frequency - 100U) * half) / 900;
  }
  return FREQ_TRACK_X0 + half +
         ((int)(frequency - 1000U) * (span - half)) / 9000;
}

/** 绘制频率滑块、频率文字和旋钮。 */
static uint16_t generator_frequency_hz = 1000U; // 当前矩形波频率

/** 绘制频率滑块、频率文字和旋钮。 */
static void draw_frequency_slider(uint16_t *framebuffer)
{
  char label[18] = "F="; // 频率文字缓冲
  char number[6];
  int knob_x = slider_x_from_frequency(generator_frequency_hz); // 旋钮位置

  format_uint(number, generator_frequency_hz);
  strcat(label, number); // 拼接数字
  strcat(label, "HZ");   // 拼接单位

  fill_rect(framebuffer, FREQ_SLIDER_X0, FREQ_SLIDER_Y0,
            FREQ_SLIDER_X1, FREQ_SLIDER_Y1, COLOR_PANEL); // 滑块背景
  draw_text(framebuffer, FREQ_SLIDER_X0 + 2, FREQ_SLIDER_Y0 + 2,
            label, COLOR_TEXT, 1); // 当前频率
  line_h(framebuffer, FREQ_TRACK_X0, FREQ_TRACK_X1,
         FREQ_SLIDER_Y0 + 19, COLOR_GRID_MAJOR); // 可拖动轨道
  fill_rect(framebuffer, knob_x - 2, FREQ_SLIDER_Y0 + 13,
            knob_x + 2, FREQ_SLIDER_Y0 + 25, COLOR_CH1); // 旋钮标记
}

/* D4 快照状态，只供保存按钮绘制和 capture_snapshot 使用。 */
static bool snapshot_ready;           // 是否已有可导出快照
static bool save_status_ok;           // 最近一次保存是否成功
static uint32_t save_status_until_ms; // 状态文字显示截止时间

/** 绘制 D4 保存按钮和最近一次保存结果。 */
static void draw_save_status(uint16_t *framebuffer)
{
  uint32_t now = HAL_GetTick();
  bool recent = (save_status_until_ms != 0U) &&
                ((int32_t)(save_status_until_ms - now) > 0); // 是否显示临时结果
  const char *label = snapshot_ready ? "SAVED" : "SAVE D4"; // 默认文字
  uint16_t color = snapshot_ready ? COLOR_OK : COLOR_MUTED; // 默认颜色

  if (recent)
  {
    label = save_status_ok ? "SAVED" : "NO DATA"; // 最近一次保存结果
    color = save_status_ok ? COLOR_OK : COLOR_ERROR; // 成功绿色/失败红色
  }

  fill_rect(framebuffer, 2, 242, 62, 266, COLOR_PANEL); // 按钮背景
  draw_text(framebuffer, 8, 249, label, color, 1);      // 保存状态
}

/* 触发参数：触发绘画面板、触发查找和控制函数共同使用。 */
typedef enum
{
  TRIGGER_OFF = 0,
  TRIGGER_A1,
  TRIGGER_A2,
  TRIGGER_D1,
  TRIGGER_D2,
  TRIGGER_D3
} TriggerSource;

#define TRIGGER_SOURCE_MASK_A1 0x01U
#define TRIGGER_SOURCE_MASK_A2 0x02U
#define TRIGGER_SOURCE_MASK_D1 0x04U
#define TRIGGER_SOURCE_MASK_D2 0x08U
#define TRIGGER_SOURCE_MASK_D3 0x10U
#define TRIGGER_SOURCE_MASK_ALL 0x1FU

static uint8_t trigger_source_mask;          // 每一位代表一个已启用触发源
static bool trigger_rising_edge = true;      // true=上升沿
static uint16_t trigger_level_mv = 1500U;    // 模拟触发电平
static uint16_t trigger_offset;               // 本帧触发点位置
static bool trigger_found;                    // 本帧是否找到触发点
static bool trigger_menu_open;                // 触发面板是否打开

static const char *trigger_source_name(TriggerSource source);

static uint8_t trigger_source_bit(TriggerSource source)
{
  if ((source <= TRIGGER_OFF) || (source > TRIGGER_D3))
  {
    return 0U;
  }
  return (uint8_t)(1U << ((uint8_t)source - 1U));
}

static bool trigger_source_enabled(TriggerSource source)
{
  uint8_t bit = trigger_source_bit(source);
  return (bit != 0U) && ((trigger_source_mask & bit) != 0U);
}

static bool trigger_any_source_enabled(void)
{
  return trigger_source_mask != 0U;
}

static uint8_t trigger_enabled_count(void)
{
  uint8_t count = 0U;
  for (uint8_t bit = 0U; bit < 5U; ++bit)
  {
    if ((trigger_source_mask & (1U << bit)) != 0U)
    {
      ++count;
    }
  }
  return count;
}

static void format_trigger_source_summary(char *output, uint8_t capacity)
{
  uint8_t count = trigger_enabled_count();

  output[0] = '\0';
  if (count == 0U)
  {
    strncpy(output, "OFF", capacity - 1U);
  }
  else if (count >= 3U)
  {
    char number[4];
    format_uint(number, count);
    strncpy(output, number, capacity - 1U);
    strncat(output, "CH", capacity - strlen(output) - 1U);
  }
  else
  {
    for (uint8_t index = TRIGGER_A1; index <= TRIGGER_D3; ++index)
    {
      if (!trigger_source_enabled((TriggerSource)index))
      {
        continue;
      }
      if (output[0] != '\0')
      {
        strncat(output, "+", capacity - strlen(output) - 1U);
      }
      strncat(output, trigger_source_name((TriggerSource)index),
              capacity - strlen(output) - 1U);
    }
  }
  output[capacity - 1U] = '\0';
}

/** 绘制底部 TRG 按钮及当前触发参数摘要。 */
static void draw_trigger_button(uint16_t *framebuffer)
{
  char label[20] = "TRG:";
  char sources[10];

  format_trigger_source_summary(sources, sizeof(sources));
  strcat(label, sources);
  strcat(label, trigger_rising_edge ? " R" : " F");
  if (trigger_source_mask & (TRIGGER_SOURCE_MASK_A1 | TRIGGER_SOURCE_MASK_A2))
  {
    char number[8];
    format_uint(number, trigger_level_mv);
    strcat(label, number);
  }

  fill_rect(framebuffer, 68, 242, 178, 266,
            trigger_menu_open ? COLOR_WARN : COLOR_PANEL);
  draw_text(framebuffer, 74, 249, label,
            trigger_any_source_enabled() ? COLOR_TEXT : COLOR_MUTED, 1);
}

/** 绘制触发源、边沿、电平和关闭按钮组成的设置面板。 */
static void draw_trigger_config(uint16_t *framebuffer)
{
  static const char *const sources[] =
      {"OFF", "A1", "A2", "D1", "D2", "D3"};
  char level_text[18] = "LEVEL ";
  char number[8];

  fill_rect(framebuffer, 34, 72, 446, 230, COLOR_PANEL); // 面板背景
  draw_text(framebuffer, 48, 80, "TRIGGER SETUP", COLOR_TEXT, 1); // 标题

  /* 第一行：触发源按钮。 */
  for (uint8_t index = 0U; index < 6U; ++index)
  {
    int x0 = 48 + (int)index * 62; // 当前按钮左边界
    bool selected =
        (index == TRIGGER_OFF) ? !trigger_any_source_enabled() :
                                 trigger_source_enabled((TriggerSource)index);
    fill_rect(framebuffer, x0, 98, x0 + 54, 120,
              selected ? COLOR_CH1 : COLOR_BACKGROUND); // 选中高亮
    draw_text(framebuffer, x0 + 14, 104, sources[index],
              selected ? COLOR_BACKGROUND : COLOR_TEXT, 1); // 文字颜色
  }

  /* 第二行：上升沿/下降沿选择。 */
  fill_rect(framebuffer, 48, 132, 116, 154,
            trigger_rising_edge ? COLOR_CH1 : COLOR_BACKGROUND);
  draw_text(framebuffer, 68, 138, "RISE",
            trigger_rising_edge ? COLOR_BACKGROUND : COLOR_TEXT, 1);
  fill_rect(framebuffer, 122, 132, 190, 154,
            !trigger_rising_edge ? COLOR_CH1 : COLOR_BACKGROUND);
  draw_text(framebuffer, 140, 138, "FALL",
            !trigger_rising_edge ? COLOR_BACKGROUND : COLOR_TEXT, 1);

  /* 第三行：触发电平粗调和细调按钮。 */
  fill_rect(framebuffer, 48, 166, 104, 188, COLOR_BACKGROUND);
  draw_text(framebuffer, 54, 172, "-500", COLOR_TEXT, 1);
  fill_rect(framebuffer, 110, 166, 166, 188, COLOR_BACKGROUND);
  draw_text(framebuffer, 116, 172, "-100", COLOR_TEXT, 1);
  fill_rect(framebuffer, 292, 166, 348, 188, COLOR_BACKGROUND);
  draw_text(framebuffer, 298, 172, "+100", COLOR_TEXT, 1);
  fill_rect(framebuffer, 354, 166, 410, 188, COLOR_BACKGROUND);
  draw_text(framebuffer, 360, 172, "+500", COLOR_TEXT, 1);

  format_uint(number, trigger_level_mv); // 当前电平数字
  strcat(level_text, number);
  strcat(level_text, "MV");
  draw_text(framebuffer, 174, 172, level_text, COLOR_CH2, 1); // 当前电平

  fill_rect(framebuffer, 380, 198, 438, 222, COLOR_OK); // 关闭按钮
  draw_text(framebuffer, 392, 205, "CLOSE", COLOR_BACKGROUND, 1); // 关闭文字
}

/** 绘制模拟区网格、数字通道分隔线和所有静态按钮。 */
static void draw_grid(uint16_t *framebuffer)
{
  uint8_t i;

  /* 先铺满背景，再覆盖顶部、模拟区和数字区三个面板。 */
  fill_rect(framebuffer, 0, 0, (int)LCD_WIDTH - 1,
            (int)LCD_HEIGHT - 1, COLOR_BACKGROUND);
  fill_rect(framebuffer, 0, 0, (int)LCD_WIDTH - 1, 38, COLOR_PANEL); // 顶部按钮区
  fill_rect(framebuffer, (int)PLOT_LEFT, (int)PLOT_TOP,
            (int)PLOT_RIGHT, ANALOG_PLOT_BOTTOM, COLOR_PLOT); // 模拟波形底色
  fill_rect(framebuffer, 0, DIGITAL_AREA_TOP,
            (int)LCD_WIDTH - 1, (int)PLOT_BOTTOM, COLOR_PANEL); // 数字波形区

  /* 10 个水平时间格。 */
  for (i = 0; i <= 10U; ++i)
  {
    int x = PLOT_LEFT + ((int)i * (PLOT_WIDTH - 1)) / 10; // 第 i 条时间线
    line_v(framebuffer, x, PLOT_TOP, PLOT_BOTTOM,
           (i == 0U || i == 10U) ? COLOR_GRID_MAJOR : COLOR_GRID); // 边界加亮
  }

  /* 四等分纵向电压刻度，对应 3.0 / 1.5 / 0 V。 */
  for (i = 0; i <= 4U; ++i)
  {
    int y = PLOT_TOP + ((int)i * (PLOT_HEIGHT - 1)) / 4; // 电压刻度线
    line_h(framebuffer, PLOT_LEFT, PLOT_RIGHT, y,
           (i == 0U || i == 4U) ? COLOR_GRID_MAJOR : COLOR_GRID); // 上下边界加亮
  }

  for (i = 0U; i < DIGITAL_CHANNEL_COUNT; ++i)
  {
    int y = DIGITAL_AREA_TOP + 7 + ((int)i * DIGITAL_LANE_HEIGHT); // 当前数字通道中心
    line_h(framebuffer, 0, LCD_WIDTH - 1, y + DIGITAL_LANE_HEIGHT - 4,
           COLOR_GRID);
    draw_text(framebuffer, 5, y - 3, channel_labels[2U + i], // D1/D2/D3 标签
              channel_color(2U + i), 1);
  }

  draw_text(framebuffer, 4, 82, "3.0", COLOR_MUTED, 1);
  draw_text(framebuffer, 4, 123, "1.5", COLOR_MUTED, 1);
  draw_text(framebuffer, 4, 161, "0.0", COLOR_MUTED, 1);
  draw_text(framebuffer, 4, 4, "T/DIV", COLOR_MUTED, 1);
  /* 网格完成后叠加所有交互按钮。 */
  draw_timebase_buttons(framebuffer);
  draw_channel_buttons(framebuffer);
  draw_frequency_slider(framebuffer);
  draw_save_status(framebuffer);
  draw_trigger_button(framebuffer);
}

/**
 * 把模拟量采样值绘制成折线；点少时插值，点多时保留每列极值。
 */
static void draw_waveform(uint16_t *framebuffer, const uint16_t *data,
                          uint16_t count, uint16_t color) // 画布/数据/点数/颜色
{
  int x;
  int previous_center = PLOT_BOTTOM;
  bool have_previous = false;

  if ((data == NULL) || (count == 0U))
  {
    return;
  }

  /*
   * CH2经过显示点数除以2后，250个点少于绘图区像素数。若仍按“一列重复
   * 一个采样点”绘制，正弦波和方波都会出现明显水平小台阶。这里用相邻
   * 采样点做线性插值：麦克风曲线变平滑，方波跳变仍只跨越很短的水平距离。
   */
  if (count < PLOT_WIDTH)
  {
    int previous_x = PLOT_LEFT; // 上一个绘制列
    int previous_center;        // 上一列中心 Y
    uint16_t first_value = (data[0] > 3000U) ? 3000U : data[0]; // 限制首点范围

    previous_center = ANALOG_PLOT_BOTTOM - // 首个采样点转屏幕 Y
                      ((int)first_value * (PLOT_HEIGHT - 1)) / 3000;
    pixel_put(framebuffer, PLOT_LEFT, previous_center, color);

    for (int column = PLOT_LEFT + 1; column <= PLOT_RIGHT; ++column)
    {
      uint64_t position_q16 =
          ((uint64_t)(column - PLOT_LEFT) * (count - 1U) * 65536ULL) /
          (PLOT_WIDTH - 1U); // 16.16 定点坐标，用于线性插值
      uint16_t index = (uint16_t)(position_q16 >> 16U);    // 左端点索引
      uint16_t fraction = (uint16_t)(position_q16 & 0xFFFFU); // 小数权重
      uint16_t next_index;
      uint32_t value;
      int y;

      if (index >= (uint16_t)(count - 1U))
      {
        index = (uint16_t)(count - 2U);
        fraction = 0xFFFFU; // 最后一列取最后一个采样点
      }
      next_index = (uint16_t)(index + 1U); // 右端点索引
      value =
          (((uint32_t)data[index] * (65536UL - fraction)) +
           ((uint32_t)data[next_index] * fraction) + 32768UL) >> 16U;
      if (value > 3000U) value = 3000U; // 防止超出显示范围
      y = ANALOG_PLOT_BOTTOM - ((int)value * (PLOT_HEIGHT - 1)) / 3000; // mV 转屏幕 Y

      line_segment(framebuffer, previous_x, previous_center,
                   column, y, color);
      previous_x = column;  // 下一段从当前列开始
      previous_center = y;  // 保存当前中心高度
    }
    return;
  }

  /*
   * 每列屏幕像素可能对应多个采样点。
   * 计算这一列中的最小值和最大值，既能保留窄脉冲，也能避免漏采样。
   */
  for (x = PLOT_LEFT + 1; x <= PLOT_RIGHT; ++x)
  {
    uint16_t first = (uint16_t)(((uint32_t)(x - PLOT_LEFT) * count) /
                                PLOT_WIDTH);
    uint16_t last = (uint16_t)(((uint32_t)(x - PLOT_LEFT + 1U) * count) /
                               PLOT_WIDTH);
    uint16_t minimum;
    uint16_t maximum;
    uint16_t index;
    int y_top;
    int y_bottom;

    if (first >= count) first = count - 1U;
    if (last <= first) last = first + 1U;
    if (last > count) last = count;
    minimum = data[first];
    maximum = data[first];
    for (index = first + 1U; index < last; ++index)
    {
      if (data[index] < minimum) minimum = data[index]; // 保留窄脉冲低点
      if (data[index] > maximum) maximum = data[index]; // 保留窄脉冲高点
    }

    if (maximum > 3000U) maximum = 3000U;
    if (minimum > 3000U) minimum = 3000U;

    y_top = ANALOG_PLOT_BOTTOM -
            ((int)maximum * (PLOT_HEIGHT - 1)) / 3000;
    y_bottom = ANALOG_PLOT_BOTTOM -
               ((int)minimum * (PLOT_HEIGHT - 1)) / 3000;
    if (y_bottom == y_top)
    {
      pixel_put(framebuffer, x, y_top, color);
    }
    else
    {
      line_v(framebuffer, x, y_top, y_bottom, color);
    }

    /*
     * The min/max column preserves narrow spikes, while this link prevents
     * steep or sparse sampling from appearing as disconnected vertical bars.
     */
    if (have_previous)
    {
      line_segment(framebuffer, x - 1, previous_center,
                   x, (y_top + y_bottom) / 2, color);
    }
    previous_center = (y_top + y_bottom) / 2;
    have_previous = true;
  }
}

/* 波形数据：原始通道数据、触发后数据以及当前帧统计。 */
static uint16_t samples_ch1[APP_FRAME_MAX_SAMPLES];          // CH1 原始点
static uint16_t samples_ch2[APP_FRAME_MAX_SAMPLES];          // CH2 原始点
static uint8_t samples_digital[APP_FRAME_MAX_SAMPLES];       // D1~D3 位图
static uint16_t triggered_samples_ch1[APP_FRAME_MAX_SAMPLES];// 触发后 CH1
static uint16_t triggered_samples_ch2[APP_FRAME_MAX_SAMPLES];// 触发后 CH2
static uint8_t triggered_samples_digital[APP_FRAME_MAX_SAMPLES]; // 触发后数字量
static uint16_t sample_count_ch1;       // CH1 有效点数
static uint16_t sample_count_ch2;       // CH2 有效点数
static uint16_t sample_count_digital;   // 数字通道有效点数
static uint16_t sample_interval_us;     // 当前采样间隔
static uint16_t frame_sequence;         // 当前帧序号
static uint16_t digital_scroll_offsets[DIGITAL_CHANNEL_COUNT]; // 三路数字波形各自的滚动偏移

/**
 * 将三路数字通道的滚动位置全部清零。
 * 修改触发源或重新初始化时调用，避免旧偏移影响新显示。
 */
static void reset_digital_scroll_offsets(void)
{
  for (uint8_t index = 0U; index < DIGITAL_CHANNEL_COUNT; ++index)
  {
    digital_scroll_offsets[index] = 0U;
  }
}

static const char *trigger_source_name(TriggerSource source)
{
  switch (source)
  {
    case TRIGGER_A1: return "A1";
    case TRIGGER_A2: return "A2";
    case TRIGGER_D1: return "D1";
    case TRIGGER_D2: return "D2";
    case TRIGGER_D3: return "D3";
    default: return "OFF";
  }
}

/**
 * 查找指定边沿，并把所有通道按同一偏移循环搬移。
 */
static bool prepare_triggered_frame(void)
{
  uint16_t count = sample_count_ch1;
  uint8_t digital_mask = 0U;

  trigger_found = false; // 每帧都重新查找
  trigger_offset = 0U;   // 默认不搬移
  if (!trigger_any_source_enabled() || count < 2U)
  {
    return false;
  }

  if ((trigger_source_mask & TRIGGER_SOURCE_MASK_A2) &&
      (sample_count_ch2 < count))
  {
    count = sample_count_ch2; // A2 点数不足时使用较短长度
  }
  if ((trigger_source_mask &
       (TRIGGER_SOURCE_MASK_D1 | TRIGGER_SOURCE_MASK_D2 |
        TRIGGER_SOURCE_MASK_D3)) &&
      (sample_count_digital < count))
  {
    count = sample_count_digital; // 数字通道点数不足时使用较短长度
  }
  if (count < 2U)
  {
    return false;
  }

  if ((trigger_source_mask & TRIGGER_SOURCE_MASK_D1) != 0U) digital_mask |= 0x01U;
  if ((trigger_source_mask & TRIGGER_SOURCE_MASK_D2) != 0U) digital_mask |= 0x02U;
  if ((trigger_source_mask & TRIGGER_SOURCE_MASK_D3) != 0U) digital_mask |= 0x04U;

  /* 扫描相邻采样点，只接受第一次符合方向的边沿。 */
  for (uint16_t index = 1U; index < count; ++index)
  {
    bool crossed = false;

    if (digital_mask != 0U) // 数字触发
    {
      uint8_t previous = samples_digital[index - 1U] & digital_mask;
      uint8_t current = samples_digital[index] & digital_mask;
      crossed = trigger_rising_edge
                    ? ((uint8_t)(~previous & current) != 0U)
                    : ((uint8_t)(previous & (uint8_t)~current) != 0U);
    }

    if (!crossed &&
        (trigger_source_mask & (TRIGGER_SOURCE_MASK_A1 |
                                TRIGGER_SOURCE_MASK_A2)))
    {
      if (trigger_source_enabled(TRIGGER_A1))
      {
        crossed = trigger_rising_edge
                      ? ((samples_ch1[index - 1U] < trigger_level_mv) &&
                         (samples_ch1[index] >= trigger_level_mv))
                      : ((samples_ch1[index - 1U] > trigger_level_mv) &&
                         (samples_ch1[index] <= trigger_level_mv));
      }
      if (!crossed && trigger_source_enabled(TRIGGER_A2))
      {
        crossed = trigger_rising_edge
                      ? ((samples_ch2[index - 1U] < trigger_level_mv) &&
                         (samples_ch2[index] >= trigger_level_mv))
                      : ((samples_ch2[index - 1U] > trigger_level_mv) &&
                         (samples_ch2[index] <= trigger_level_mv));
      }
    }

    if (crossed)
    {
      trigger_offset = index; // 记住触发点
      trigger_found = true;   // 标记本帧触发成功
      break;
    }
  }

  if (!trigger_found)
  {
    return false;
  }

  /* 按触发偏移循环读取，让触发点移动到数组第一个位置。 */
  for (uint16_t index = 0U; index < count; ++index)
  {
    uint16_t source_index = (uint16_t)(trigger_offset + index);
    if (source_index >= count)
    {
      source_index = (uint16_t)(source_index - count); // 循环搬移
    }
    triggered_samples_ch1[index] = samples_ch1[source_index];         // 同一偏移
    triggered_samples_ch2[index] = samples_ch2[source_index];         // 保持同步
    triggered_samples_digital[index] = samples_digital[source_index]; // 保持同步
  }
  return true;
}

/* 测量结果结构：紧随测量函数放置，便于对照字段含义。 */
typedef struct
{
  uint16_t peak_mv;       // 峰值电压
  uint32_t frequency_hz;  // 估算频率
} AnalogMeasurement;

typedef struct
{
  uint32_t frequency_hz;   // 数字信号频率
  uint16_t duty_permille;  // 占空比，千分比
  uint16_t edge_count;     // 上升沿数量
  uint16_t sample_span;    // 首末边沿间距
} DigitalMeasurement;

static AnalogMeasurement analog_measurements[2];              // CH1/CH2
static DigitalMeasurement digital_measurements[DIGITAL_CHANNEL_COUNT];

/**
 * 通过迟滞过零检测计算模拟波形频率，并记录峰值电压。
 */
static void measure_analog_channel(const uint16_t *data, uint16_t count,
                                   uint16_t interval_us,
                                   AnalogMeasurement *measurement) // 输出测量结果
{
  uint16_t minimum = 0xFFFFU;
  uint16_t maximum = 0U;
  int32_t center;
  int32_t hysteresis;
  int32_t low_threshold;
  int32_t high_threshold;
  bool armed;
  uint32_t crossing_count = 0U;
  int32_t first_crossing_q16 = 0;
  int32_t last_crossing_q16 = 0;

  measurement->peak_mv = 0U;     // 先清空上次测量结果
  measurement->frequency_hz = 0U;
  if ((data == NULL) || (count < 2U) || (interval_us == 0U))
  {
    return;
  }

  /* 第一步：寻找整帧最小值和最大值。 */
  for (uint16_t index = 0U; index < count; ++index)
  {
    if (data[index] < minimum) minimum = data[index];
    if (data[index] > maximum) maximum = data[index];
  }
  measurement->peak_mv = maximum;
  if ((uint16_t)(maximum - minimum) < 12U) // 变化太小，视为无有效信号
  {
    return;
  }

  center = ((int32_t)minimum + (int32_t)maximum) / 2; // 波形中心电平
  hysteresis = (int32_t)(maximum - minimum) / 8;      // 峰峰值的 1/8 作为迟滞
  if (hysteresis < 6) hysteresis = 6;                 // 防止小信号频繁误触发
  low_threshold = center - hysteresis;                // 重新准备触发的低阈值
  high_threshold = center + hysteresis;               // 确认上升沿的高阈值
  armed = ((int32_t)data[0] <= low_threshold);        // 从低电平开始才允许计数

  /* 第二步：用低、高两个阈值避免噪声造成重复计数。 */
  for (uint16_t index = 1U; index < count; ++index)
  {
    int32_t previous = data[index - 1U];
    int32_t current = data[index];

    if (!armed && (current <= low_threshold))
    {
      armed = true; // 已回到低阈值，准备下一次上升沿
    }
    if (armed && (current >= high_threshold) &&
        (current > previous))
    {
      int32_t crossing_q16 =
          ((int32_t)(index - 1U) * 65536) +
          (((center - previous) * 65536) / (current - previous)); // 插值得到过零点
      if (crossing_count == 0U)
      {
        first_crossing_q16 = crossing_q16;
      }
      last_crossing_q16 = crossing_q16;
      ++crossing_count; // 记录一次有效过零
      armed = false;    // 等待波形再次回到低阈值
    }
  }

  if ((crossing_count >= 2U) && (last_crossing_q16 > first_crossing_q16))
  {
    /* 多次同向过零之间的平均间隔用于计算频率。 */
    uint64_t numerator =
        (uint64_t)(crossing_count - 1U) * 65536ULL * 1000000ULL;
    uint64_t denominator =
        (uint64_t)(last_crossing_q16 - first_crossing_q16) *
        interval_us;
    measurement->frequency_hz =
        (uint32_t)((numerator + denominator / 2U) / denominator); // 过零频率
    /*
     * The analog input path produces two like-direction zero crossings per
     * signal cycle on the hardware setup, so normalize to the source period.
     */
    measurement->frequency_hz /= ANALOG_FREQUENCY_DIVISOR;        // 换算为信号频率
  }
}

/**
 * 统计一路数字信号的频率、占空比和边沿数量。
 */
static void measure_digital_channel(uint8_t bit_mask, uint16_t interval_us,
                                    DigitalMeasurement *measurement) // 输出测量结果
{
  uint16_t first_rising = 0U;
  uint16_t last_rising = 0U;
  uint16_t rising_count = 0U;
  uint16_t high_count = 0U;
  uint16_t total_high = 0U;

  measurement->frequency_hz = 0U;   // 清空旧测量值
  measurement->duty_permille = 0U;
  measurement->edge_count = 0U;
  measurement->sample_span = 0U;
  if ((interval_us == 0U) || (sample_count_digital < 2U))
  {
    return;
  }

  /* 先按整帧统计高电平比例。 */
  for (uint16_t index = 0U; index < sample_count_digital; ++index)
  {
    if ((samples_digital[index] & bit_mask) != 0U)
    {
      ++total_high; // 统计高电平时长
    }
  }
  measurement->duty_permille =
      (uint16_t)(((uint32_t)total_high * 1000U + sample_count_digital / 2U) /
                 sample_count_digital);

  /* 记录第一个和最后一个上升沿。 */
  for (uint16_t index = 1U; index < sample_count_digital; ++index)
  {
    bool previous_high =
        (samples_digital[index - 1U] & bit_mask) != 0U;
    bool current_high = (samples_digital[index] & bit_mask) != 0U;
    if (!previous_high && current_high)
    {
      if (rising_count == 0U) first_rising = index; // 第一个上升沿
      last_rising = index;                          // 最新上升沿
      ++rising_count;                               // 累计上升沿个数
    }
  }

  if ((rising_count >= 2U) && (last_rising > first_rising))
  {
    uint32_t span = (uint32_t)(last_rising - first_rising);
    uint64_t denominator = (uint64_t)span * interval_us; // 总时间
    uint64_t numerator = (uint64_t)(rising_count - 1U) * 1000000ULL; // 周期数

    measurement->frequency_hz =
        (uint32_t)((numerator + denominator / 2U) / denominator);
    measurement->edge_count = rising_count;    // 调试用边沿数
    measurement->sample_span = (uint16_t)span; // 首末边沿采样距离
    /* 用首末上升沿之间的数据重新计算占空比。 */
    for (uint16_t index = first_rising; index < last_rising; ++index)
    {
      if ((samples_digital[index] & bit_mask) != 0U)
      {
        ++high_count;
      }
    }
    measurement->duty_permille =
        (uint16_t)(((uint32_t)high_count * 1000U + span / 2U) / span);
  }
}

/**
 * 将一路数字采样位图绘制成高低电平方波。
 */
static void draw_digital_waveform(uint16_t *framebuffer,
                                  const uint8_t *digital_data,
                                  uint8_t bit_mask, uint8_t digital_index,
                                  uint16_t display_offset) // 数据/掩码/通道号/滚动偏移
{
  int lane_y = DIGITAL_AREA_TOP + 7 +
               ((int)digital_index * DIGITAL_LANE_HEIGHT);
  int y_high = lane_y - 5;
  int y_low = lane_y + 5;
  int previous_center = y_low;
  bool have_previous = false;

  if (!channel_visible[2U + digital_index] || (sample_count_digital == 0U))
  {
    return;
  }

  for (int x = PLOT_LEFT + 1; x <= PLOT_RIGHT; ++x)
  {
    uint16_t first = (uint16_t)(((uint32_t)(x - PLOT_LEFT) *
                                 sample_count_digital) / PLOT_WIDTH);
    uint16_t last = (uint16_t)(((uint32_t)(x - PLOT_LEFT + 1U) *
                                sample_count_digital) / PLOT_WIDTH);
    bool any_high = false; // 这一列是否出现高电平
    bool any_low = false;  // 这一列是否出现低电平
    int y;

    if (first >= sample_count_digital) first = sample_count_digital - 1U;
    if (last <= first) last = first + 1U;
    if (last > sample_count_digital) last = sample_count_digital;

    for (uint16_t index = first; index < last; ++index)
    {
      uint16_t source_index =
          (uint16_t)(index + display_offset);
      if (source_index >= sample_count_digital)
      {
        source_index = (uint16_t)(source_index - sample_count_digital); // 循环滚动
      }
      if ((digital_data[source_index] & bit_mask) != 0U)
      {
        any_high = true; // 保留窄高脉冲
      }
      else
      {
        any_low = true; // 该列同时出现跳变
      }
    }

    if (any_high && any_low)
    {
      line_v(framebuffer, x, y_high, y_low,
             channel_color(2U + digital_index));
      y = (y_high + y_low) / 2;
    }
    else
    {
      y = any_high ? y_high : y_low;
      pixel_put(framebuffer, x, y, channel_color(2U + digital_index));
    }

    if (have_previous)
    {
      line_v(framebuffer, x, previous_center, y,
             channel_color(2U + digital_index));
    }
    previous_center = y;
    have_previous = true;
  }
}

/** 绘制一个模拟通道的峰值和频率。 */
static void draw_analog_measurement(uint16_t *framebuffer, int label_x,
                                    const char *label, uint16_t color,
                                    const AnalogMeasurement *measurement)
{
  char line[32];   // 最终显示文字
  char number[12]; // 临时数字字符串

  strcpy(line, label);
  if (measurement->peak_mv > 0U)
  {
    format_voltage(measurement->peak_mv, number); // 峰值转 x.xxxV
    strcat(line, " P=");                           // 峰值标记
    strcat(line, number);
    format_uint(number, measurement->frequency_hz); // 频率转十进制
    strcat(line, " F=");                             // 频率标记
    strcat(line, number);
    strcat(line, "HZ");
  }
  else
  {
    strcat(line, " P=--- F=---");
  }
  draw_text(framebuffer, label_x, 13, line, color, 1); // 绘制到顶部
}

/** 绘制三路数字通道的频率和占空比。 */
static void draw_digital_measurements(uint16_t *framebuffer)
{
  for (uint8_t index = 0U; index < DIGITAL_CHANNEL_COUNT; ++index)
  {
    int lane_y = DIGITAL_AREA_TOP + 7 + ((int)index * DIGITAL_LANE_HEIGHT); // 通道中心
    char line[24] = "D";             // 以 D 开头
    char number[12];                 // 临时数字
    uint16_t color = channel_color(2U + index); // 对应通道颜色

    line[1] = (char)('1' + index); // 生成 D1/D2/D3
    strcat(line, " ");
    if (sample_count_digital == 0U)
    {
      strcat(line, "---");
    }
    else
    {
      format_uint(number, digital_measurements[index].frequency_hz); // 频率
      strcat(line, number);
      strcat(line, "HZ ");
      format_uint(number, digital_measurements[index].duty_permille / 10U); // 百分比
      strcat(line, number);
      strcat(line, "%");
    }

    fill_rect(framebuffer, 2, lane_y - 8, 116, lane_y + 8, COLOR_PANEL); // 标签底色
    draw_text(framebuffer, 5, lane_y - 6, line, color, 1);
  }
}

/* 接收状态栏和暂停状态，紧挨着状态绘制函数。 */
typedef enum
{
  RX_WAITING = 0,
  RX_FRAME_OK,
  RX_CRC_ERROR,
  RX_FORMAT_ERROR
} RxVisualState;

static RxVisualState visual_state = RX_WAITING; // 当前接收状态
static bool waveform_paused;                    // 波形是否暂停

/** 绘制接收状态、采样间隔和暂停提示。 */
static void draw_status(uint16_t *framebuffer, RxVisualState state,
                        const char *detail)
{
  char rate_text[18] = "SAMPLE ---";
  uint16_t color;
  const char *label;

  (void)detail;

  switch (state) // 根据接收状态选择文字和颜色
  {
    case RX_FRAME_OK:
      color = COLOR_OK;
      label = "RX OK";
      break;
    case RX_CRC_ERROR:
      color = COLOR_ERROR;
      label = "CRC ERROR";
      break;
    case RX_FORMAT_ERROR:
      color = COLOR_WARN;
      label = "BAD FRAME";
      break;
    case RX_WAITING:
    default:
      color = COLOR_WARN;
      label = "WAIT RX";
      break;
  }

  draw_text(framebuffer, 393, 248, label, color, 1);

  if (sample_interval_us > 0U)
  {
    char number[11];

    strcpy(rate_text, "SAMPLE ");
    format_uint(number, sample_interval_us);
    strcat(rate_text, number);
    strcat(rate_text, "US");
  }
  draw_text(framebuffer, 184, 248, rate_text,
            timebase_pending ? COLOR_WARN :
            (timebase_failed ? COLOR_ERROR : COLOR_MUTED), 1);
  if (waveform_paused)
  {
    draw_text(framebuffer, 286, 248, "PAUSED", COLOR_WARN, 1);
  }
  else if (timebase_failed)
  {
    draw_text(framebuffer, 286, 248, "TB FAIL", COLOR_ERROR, 1);
  }
}

/* 双缓冲屏幕内存：前后台缓冲区交替绘制和显示。 */
static uint16_t *const framebuffers[2] = {
    (uint16_t *)LCD_FB0_ADDRESS, // 前台/后台 0
    (uint16_t *)LCD_FB1_ADDRESS, // 前台/后台 1
};
static uint16_t *const background_framebuffer =
    (uint16_t *)LCD_BACKGROUND_ADDRESS; // 静态背景副本
static uint8_t active_framebuffer;      // 当前显示缓冲区编号

/** 把指定 SDRAM 帧缓冲区交给 LTDC 显示。 */
static void present_frame(uint16_t *framebuffer)
{
  (void)HAL_LTDC_SetAddress(&hltdc, (uint32_t)framebuffer, 0); // 切换图层 0 显存
}

/** 绘制尚未收到板间数据时的等待界面。 */
static void render_waiting_screen(void)
{
  uint16_t *framebuffer = framebuffers[active_framebuffer]; // 当前前台缓冲区

  memcpy(framebuffer, background_framebuffer, // 复制静态背景
         LCD_FRAME_PIXELS * sizeof(uint16_t));
  draw_timebase_buttons(framebuffer);
  draw_channel_buttons(framebuffer);
  draw_frequency_slider(framebuffer);
  draw_save_status(framebuffer);
  draw_trigger_button(framebuffer);
  draw_analog_measurement(framebuffer, 8, "CH1",
                          COLOR_CH1, &analog_measurements[0]);
  draw_analog_measurement(framebuffer, 150, "CH2",
                          COLOR_CH2, &analog_measurements[1]);
  draw_text(framebuffer, 112, 142, "WAITING FOR DATA", COLOR_WARN, 1); // 等待提示
  draw_status(framebuffer, RX_WAITING, NULL); // 底部 WAIT RX
  if (trigger_menu_open)
  {
    draw_trigger_config(framebuffer);
  }
  present_frame(framebuffer); // 立即显示等待界面
}

/**
 * 绘制一帧完整波形，并在绘制完成后切换前后台缓冲区。
 */
static void render_received_frame(void)
{
  char trigger_text[20] = "TRG:OFF";
  uint16_t ch2_display_count =
      (uint16_t)(sample_count_ch2 / CH2_DISPLAY_POINT_DIVISOR);
  const uint16_t *render_ch1 = samples_ch1;
  const uint16_t *render_ch2 = samples_ch2;
  bool trigger_active = prepare_triggered_frame(); // 先尝试找到触发边沿
  /* 当前前台缓冲区保持显示，新的完整帧画到后台缓冲区。 */
  uint16_t *framebuffer = framebuffers[active_framebuffer ^ 1U]; // 后台缓冲区

  /**
   * 如果找到触发边沿，后续所有通道都改用已经循环搬移过的数据，
   * 这样模拟波形和数字波形仍保持同一时间关系。
   */
  if (trigger_active) // 使用触发后的同步数据
  {
    render_ch1 = triggered_samples_ch1;
    render_ch2 = triggered_samples_ch2;
  }

  /**
   * 先把静态背景和所有按钮复制到后台缓冲区，再叠加实时波形。
   */
  /* 先恢复背景和按钮，再绘制实时数据。 */
  memcpy(framebuffer, background_framebuffer,
         LCD_FRAME_PIXELS * sizeof(uint16_t));
  draw_timebase_buttons(framebuffer);
  draw_channel_buttons(framebuffer);
  draw_frequency_slider(framebuffer);
  draw_save_status(framebuffer);
  draw_trigger_button(framebuffer);

  /**
   * 只有 channel_visible 为真的通道才绘制波形。
   */
  if (channel_visible[0])
  {
    draw_waveform(framebuffer, render_ch1, sample_count_ch1, COLOR_CH1);
  }
  if (channel_visible[1])
  {
    draw_waveform(framebuffer, render_ch2, ch2_display_count, COLOR_CH2);
  }
  /**
   * 三路数字通道分别决定是否冻结：
   * 只有当前触发源正好是该数字通道且本帧找到触发边沿时，
   * 该通道使用触发对齐数据；另外两路继续使用原始数据滚动。
   */
  for (uint8_t channel = 0U; channel < DIGITAL_CHANNEL_COUNT; ++channel)
  {
    TriggerSource channel_source = (TriggerSource)(TRIGGER_D1 + channel);
    bool channel_triggered =
        trigger_active && trigger_source_enabled(channel_source);
    const uint8_t *digital_data =
        channel_triggered ? triggered_samples_digital : samples_digital;
    uint16_t display_offset = channel_triggered ? 0U : digital_scroll_offsets[channel];

    draw_digital_waveform(framebuffer, digital_data,
                          0x01U << channel, channel, display_offset);

    if (!channel_triggered && (sample_count_digital > 0U))
    {
      digital_scroll_offsets[channel] =
          (uint16_t)(digital_scroll_offsets[channel] + DIGITAL_SCROLL_STEP);
      if (digital_scroll_offsets[channel] >= sample_count_digital)
      {
        digital_scroll_offsets[channel] =
            (uint16_t)(digital_scroll_offsets[channel] - sample_count_digital);
      }
    }
  }
  draw_digital_measurements(framebuffer);
  draw_analog_measurement(framebuffer, 8, "CH1",
                          COLOR_CH1, &analog_measurements[0]);
  draw_analog_measurement(framebuffer, 150, "CH2",
                          COLOR_CH2, &analog_measurements[1]);

  /* 所有通道都关闭时显示提示。 */
  if (!channel_visible[0] && !channel_visible[1] &&
      !channel_visible[2] && !channel_visible[3] && !channel_visible[4])
  {
    draw_text(framebuffer, 150, 108, "ALL CHANNELS OFF", COLOR_WARN, 1);
  }

  /* 组装底部触发状态文字。 */
  if (trigger_any_source_enabled())
  {
    char number[8];
    char sources[10];
    strcpy(trigger_text, "TRG:");
    format_trigger_source_summary(sources, sizeof(sources));
    strcat(trigger_text, sources);
    strcat(trigger_text, trigger_rising_edge ? " R" : " F");
    if (trigger_source_mask & (TRIGGER_SOURCE_MASK_A1 | TRIGGER_SOURCE_MASK_A2))
    {
      format_uint(number, trigger_level_mv);
      strcat(trigger_text, number);
    }
  }
  draw_status(framebuffer, visual_state, trigger_text);
  /* 触发菜单最后绘制，保证覆盖在波形上方。 */
  if (trigger_menu_open)
  {
    draw_trigger_config(framebuffer);
  }

  present_frame(framebuffer); // 显示完整后台帧
  active_framebuffer ^= 1U;   // 切换前后台缓冲区
}

/* ============================ 6. 数据帧解析 ============================ */

/* 二进制帧重组和电脑文本命令缓冲，与下方状态机放在一起。 */
static uint8_t frame_buffer[APP_FRAME_MAX_SIZE]; // 当前正在拼接的帧
static uint16_t frame_length;                    // 已接收字节数
static uint16_t expected_frame_length;           // 帧头指示的总长度
static bool frame_header_valid;                  // 帧头是否已校验
static char pc_command_buffer[PC_COMMAND_MAX_LENGTH + 1U]; // 文本命令
static uint8_t pc_command_length;                           // 命令字符数

/** 计算协议使用的 CRC-16/CCITT-FALSE。 */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;
  uint32_t index;
  uint8_t bit;

  for (index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8; // 当前字节放入高 8 位
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/** 清空当前未完成的二进制帧。 */
static void reset_parser(void)
{
  frame_length = 0U;
  expected_frame_length = 0U;
  frame_header_valid = false;
}

static void append_text(char *output, uint8_t *position, uint8_t capacity,
                        const char *text)
{
  while ((*text != '\0') && (*position < (uint8_t)(capacity - 1U)))
  {
    output[(*position)++] = *text++;
  }
}

static void append_number(char *output, uint8_t *position, uint8_t capacity,
                          uint32_t value)
{
  char number[8];
  uint8_t length;
  uint8_t index = 0U;

  format_uint(number, value);
  length = (uint8_t)strlen(number);
  while ((index < length) && (*position < (uint8_t)(capacity - 1U)))
  {
    output[(*position)++] = number[index++];
  }
}

static void write_le16(uint8_t *output, uint16_t value)
{
  output[0] = (uint8_t)(value & 0xFFU);
  output[1] = (uint8_t)(value >> 8U);
}

static void write_le32(uint8_t *output, uint32_t value)
{
  output[0] = (uint8_t)(value & 0xFFU);
  output[1] = (uint8_t)((value >> 8U) & 0xFFU);
  output[2] = (uint8_t)((value >> 16U) & 0xFFU);
  output[3] = (uint8_t)(value >> 24U);
}

/* 快照使用单独数组和屏幕区域，避免被下一帧实时数据覆盖。 */
static uint16_t *const snapshot_framebuffer =
    (uint16_t *)LCD_SNAPSHOT_ADDRESS;                      // 冻结屏幕
static uint16_t snapshot_ch1[APP_FRAME_MAX_SAMPLES];       // 冻结 CH1
static uint16_t snapshot_ch2[APP_FRAME_MAX_SAMPLES];       // 冻结 CH2
static uint8_t snapshot_digital[APP_FRAME_MAX_SAMPLES];    // 冻结 D1~D3
static uint16_t snapshot_count;                            // 快照点数
static uint16_t snapshot_interval_us;                      // 快照采样间隔
static uint16_t snapshot_sequence;                         // 快照帧序号

/** 冻结当前屏幕、波形和数字通道状态作为快照。 */
static bool capture_snapshot(void)
{
  uint16_t count = sample_count_ch1;
  char line[64];
  uint8_t position = 0U;

  if (sample_count_ch2 > count) count = sample_count_ch2;
  if (sample_count_digital > count) count = sample_count_digital;

  if (count == 0U)
  {
    save_status_ok = false;
    save_status_until_ms = HAL_GetTick() + 1500U;
    BoardLink_WritePc((const uint8_t *)"SNAPSHOT,ERR,NO_DATA\r\n", 22U);
    refresh_timebase_screen();
    return false;
  }

  if (count > APP_FRAME_MAX_SAMPLES) count = APP_FRAME_MAX_SAMPLES; // 限制快照长度
  memset(snapshot_ch1, 0, sizeof(snapshot_ch1));
  memset(snapshot_ch2, 0, sizeof(snapshot_ch2));
  memset(snapshot_digital, 0, sizeof(snapshot_digital));
  memcpy(snapshot_ch1, samples_ch1, count * sizeof(uint16_t)); // 冻结 CH1
  memcpy(snapshot_ch2, samples_ch2, count * sizeof(uint16_t)); // 冻结 CH2
  memcpy(snapshot_digital, samples_digital, count * sizeof(uint8_t)); // 冻结 D1~D3
  memcpy(snapshot_framebuffer, framebuffers[active_framebuffer],
         LCD_FRAME_PIXELS * sizeof(uint16_t)); // 冻结当前屏幕

  snapshot_count = count;                         // 冻结点数
  snapshot_interval_us = sample_interval_us;      // 冻结采样间隔
  snapshot_sequence = frame_sequence;             // 冻结帧序号
  snapshot_ready = true; // CSV/BMP 可导出的标志
  save_status_ok = true;                          // 标记保存成功
  save_status_until_ms = HAL_GetTick() + 1500U;   // 状态显示 1.5 秒

  append_text(line, &position, sizeof(line), "SNAPSHOT,OK,POINTS=");
  append_number(line, &position, sizeof(line), snapshot_count);
  append_text(line, &position, sizeof(line), ",INTERVAL_US=");
  append_number(line, &position, sizeof(line), snapshot_interval_us);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
  refresh_timebase_screen();
  return true;
}

/** 把最近一次快照通过 USART1 发送成 CSV。 */
static void send_csv_snapshot(void)
{
  char line[128];
  uint8_t position;

  if (!snapshot_ready)
  {
    BoardLink_WritePc((const uint8_t *)"CSV,ERR,NO_SNAPSHOT\r\n", 23U);
    return;
  }

  /* 发送 CSV 元数据和表头。 */
  position = 0U;
  append_text(line, &position, sizeof(line), "CSV_BEGIN,POINTS=");
  append_number(line, &position, sizeof(line), snapshot_count);
  append_text(line, &position, sizeof(line), ",INTERVAL_US=");
  append_number(line, &position, sizeof(line), snapshot_interval_us);
  append_text(line, &position, sizeof(line), ",SEQUENCE=");
  append_number(line, &position, sizeof(line), snapshot_sequence);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
  BoardLink_WritePc(
      (const uint8_t *)"index,time_us,ch1_mv,ch2_mv,d1,d2,d3\r\n",
      (uint16_t)(sizeof("index,time_us,ch1_mv,ch2_mv,d1,d2,d3\r\n") - 1U));

  /* 每个采样点输出一行：时间、两路电压和三路数字电平。 */
  for (uint16_t index = 0U; index < snapshot_count; ++index)
  {
    position = 0U; // 开始拼新一行
    append_number(line, &position, sizeof(line), index);
    append_text(line, &position, sizeof(line), ","); // 字段分隔符
    append_number(line, &position, sizeof(line),
                  (uint32_t)index * snapshot_interval_us);
    append_text(line, &position, sizeof(line), ",");
    append_number(line, &position, sizeof(line), snapshot_ch1[index]);
    append_text(line, &position, sizeof(line), ",");
    append_number(line, &position, sizeof(line), snapshot_ch2[index]);
    append_text(line, &position, sizeof(line), ",");
    append_number(line, &position, sizeof(line),
                  (snapshot_digital[index] & 0x01U) ? 1U : 0U);
    append_text(line, &position, sizeof(line), ",");
    append_number(line, &position, sizeof(line),
                  (snapshot_digital[index] & 0x02U) ? 1U : 0U);
    append_text(line, &position, sizeof(line), ",");
    append_number(line, &position, sizeof(line),
                  (snapshot_digital[index] & 0x04U) ? 1U : 0U);
    append_text(line, &position, sizeof(line), "\r\n");
    BoardLink_WritePc((const uint8_t *)line, position);
  }

  /* 输出 CSV 结束标记，电脑脚本据此判断文件接收完成。 */
  position = 0U;
  append_text(line, &position, sizeof(line), "CSV_END,ROWS=");
  append_number(line, &position, sizeof(line), snapshot_count);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
}

/** 把最近一次屏幕快照通过 USART1 发送成 BMP。 */
static void send_bmp_snapshot(void)
{
  static uint8_t row[LCD_WIDTH * 2U];              // 一行 RGB565 像素
  uint8_t header[66] = {0};                        // BMP 文件头
  uint32_t pixel_bytes = LCD_FRAME_PIXELS * 2UL;   // 像素区长度
  uint32_t file_size = 66UL + pixel_bytes;         // 整个 BMP 长度
  char line[32];
  uint8_t position = 0U;

  if (!snapshot_ready)
  {
    BoardLink_WritePc((const uint8_t *)"BMP,ERR,NO_SNAPSHOT\r\n", 23U);
    return;
  }

  /* 写入 BMP 文件头字段。 */
  header[0] = 'B';                          // BMP 标识
  header[1] = 'M';                          // BMP 标识
  write_le32(&header[2], file_size);        // 文件总大小
  write_le16(&header[10], 66U);             // 像素数据偏移
  write_le32(&header[14], 40U);
  write_le32(&header[18], LCD_WIDTH);       // 图像宽度
  write_le32(&header[22], LCD_HEIGHT);      // 图像高度
  write_le16(&header[26], 1U);              // 单平面
  write_le16(&header[28], 16U);             // 每像素 16 位
  write_le32(&header[30], 3UL);             // RGB565 压缩方式
  write_le32(&header[34], pixel_bytes);     // 像素区大小
  write_le32(&header[38], 2835UL);
  write_le32(&header[42], 2835UL);
  write_le32(&header[54], 0x0000F800UL);
  write_le32(&header[58], 0x000007E0UL);
  write_le32(&header[62], 0x0000001FU);

  append_text(line, &position, sizeof(line), "BMP_BEGIN,SIZE=");
  append_number(line, &position, sizeof(line), file_size);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
  BoardLink_WritePc(header, sizeof(header));

  /* BMP 像素行从屏幕底部向上保存。 */
  for (int y = (int)LCD_HEIGHT - 1; y >= 0; --y)
  {
    const uint16_t *source = snapshot_framebuffer + ((uint32_t)y * LCD_WIDTH);
    for (int x = 0; x < (int)LCD_WIDTH; ++x)
    {
      write_le16(&row[x * 2], source[x]);
    }
    BoardLink_WritePc(row, sizeof(row)); // 发送一整行
  }

  BoardLink_WritePc((const uint8_t *)"\r\nBMP_END\r\n", 11U);
}

/* 主循环刷新标志和接收质量统计。 */
static volatile bool frame_ready;           // 是否有新帧需要刷新
static volatile uint32_t valid_frame_count; // 有效帧累计
static volatile uint32_t crc_error_count;   // CRC 错误累计

/*
 * PC 上位机 ACK:
 * ACK,<status>,<sequence>,<points>,<interval_us>,<ok>,<crc_errors>,<drops>
 *
 * status 为 OK、CRC 或 FORMAT。最后一个字段是串口环形缓冲区丢字节次数。
 */
/** 向电脑返回帧接收结果和错误统计。 */
static void send_frame_ack(const char *status, uint16_t sequence,
                           uint16_t count, uint16_t interval,
                           BoardLinkSource source) // 来源是电脑或采集板
{
  char response[96];    // ACK 文本缓冲
  uint8_t position = 0U; // 当前写入位置

  append_text(response, &position, sizeof(response), "ACK,"); // 消息类型
  append_text(response, &position, sizeof(response), status);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), sequence); // 帧序号
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), count); // 点数
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), interval); // 采样间隔
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), valid_frame_count); // 有效帧数
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), crc_error_count); // CRC 错误数
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), // 环形缓冲丢字节数
                BoardLink_GetBoardDropCount() +
                BoardLink_GetPcDropCount());
  append_text(response, &position, sizeof(response), "\r\n");

  /*
   * The acquisition board does not consume per-frame ACKs. Sending them back
   * wastes half-duplex time and can starve its 1 Mbit/s receive interrupt.
   * Keep the acknowledgement on USART1 for the PC host only.
   */
  (void)source;
  BoardLink_WritePc((const uint8_t *)response, position);
}

/** 向电脑发送 A1、A2、D1~D3 的测量结果。 */
static void send_measurement_debug(void)
{
  char line[128];
  uint8_t position = 0U;

  append_text(line, &position, sizeof(line), "MEA,A1P=");
  append_number(line, &position, sizeof(line), analog_measurements[0].peak_mv);
  append_text(line, &position, sizeof(line), ",A1F=");
  append_number(line, &position, sizeof(line), analog_measurements[0].frequency_hz);
  append_text(line, &position, sizeof(line), ",A2P=");
  append_number(line, &position, sizeof(line), analog_measurements[1].peak_mv);
  append_text(line, &position, sizeof(line), ",A2F=");
  append_number(line, &position, sizeof(line), analog_measurements[1].frequency_hz);
  for (uint8_t index = 0U; index < DIGITAL_CHANNEL_COUNT; ++index)
  {
    append_text(line, &position, sizeof(line), index == 0U ? ",D1=" : ",");
    if (index == 1U) append_text(line, &position, sizeof(line), "D2=");
    if (index == 2U) append_text(line, &position, sizeof(line), "D3=");
    append_number(line, &position, sizeof(line),
                  digital_measurements[index].frequency_hz);
    append_text(line, &position, sizeof(line), "HZ/");
    append_number(line, &position, sizeof(line),
                  digital_measurements[index].duty_permille / 10U);
    append_text(line, &position, sizeof(line), "%");
    append_text(line, &position, sizeof(line), "/E");
    append_number(line, &position, sizeof(line),
                  digital_measurements[index].edge_count);
    append_text(line, &position, sizeof(line), "/S");
    append_number(line, &position, sizeof(line),
                  digital_measurements[index].sample_span);
  }
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
}

/*
 * 向采集端发送时间档位命令。
 * packet[4] 是档位，packet[5..6] 是对应采样间隔（微秒，小端）。
 */
/** 向采集端发送扫描时间控制包。 */
static void send_timebase_command(uint8_t index)
{
  uint8_t packet[9];                                  // 9 字节控制包
  uint16_t interval = timebase_interval_us[index];    // 目标采样间隔
  uint16_t crc;                                        // CRC 校验值

  /*
   * 通知采集端切换采样间隔：200/100/20/10/2 us。
   */
  packet[0] = CONTROL_SYNC_0;                    // 同步头 1
  packet[1] = CONTROL_SYNC_1;                    // 同步头 2
  packet[2] = CONTROL_VERSION;                   // 协议版本
  packet[3] = CONTROL_CMD_TIMEBASE;              // 换档命令
  packet[4] = index;                             // 档位 0~4
  packet[5] = (uint8_t)(interval & 0xFFU);       // 间隔低字节
  packet[6] = (uint8_t)(interval >> 8);          // 间隔高字节
  crc = crc16_ccitt(packet, 7U);                 // 计算前 7 字节 CRC
  packet[7] = (uint8_t)(crc & 0xFFU);            // CRC 低字节
  packet[8] = (uint8_t)(crc >> 8);               // CRC 高字节

  /*
   * Send two forms on the board link:
   *   1. Short ASCII form for easy recovery from a lost byte.
   *   2. CRC-protected binary form as the primary protocol.
   * The acquisition parser accepts both. Sending ASCII first also prevents
   * printable CRC bytes from polluting the ASCII command accumulator.
   */
  {
    char fallback[6] = {'T', 'B', '=', (char)('0' + index), '\r', '\n'}; // 文本备用命令
    BoardLink_WriteBoard((const uint8_t *)fallback, sizeof(fallback));
  }
  BoardLink_WriteBoard(packet, sizeof(packet)); // CRC 二进制主命令
  /* Mirror the control packet to the PC for diagnostic logging. */
  BoardLink_WritePc(packet, sizeof(packet)); // 镜像给电脑便于记录
}

/** 向采集端发送矩形波频率控制包。 */
static void send_frequency_command(uint16_t frequency_hz)
{
  uint8_t packet[9]; // 频率控制包
  uint16_t crc;       // 包校验值

  if ((frequency_hz < FREQ_CONTROL_MIN) ||
      (frequency_hz > FREQ_CONTROL_MAX))
  {
    return;
  }

  packet[0] = CONTROL_SYNC_0;                    // 同步头 1
  packet[1] = CONTROL_SYNC_1;                    // 同步头 2
  packet[2] = CONTROL_VERSION;                   // 协议版本
  packet[3] = CONTROL_CMD_FREQUENCY;             // 设置频率命令
  packet[4] = (uint8_t)(frequency_hz & 0xFFU);   // 频率低字节
  packet[5] = (uint8_t)(frequency_hz >> 8U);     // 频率高字节
  packet[6] = 0U;                                // 保留字节
  crc = crc16_ccitt(packet, 7U);                 // 计算 CRC
  packet[7] = (uint8_t)(crc & 0xFFU);            // CRC 低字节
  packet[8] = (uint8_t)(crc >> 8U);              // CRC 高字节

  BoardLink_WriteBoard(packet, sizeof(packet));

  {
    char line[32];
    uint8_t position = 0U;
    append_text(line, &position, sizeof(line), "FREQ REQ=");
    append_number(line, &position, sizeof(line), frequency_hz);
    append_text(line, &position, sizeof(line), "\r\n");
    BoardLink_WritePc((const uint8_t *)line, position);
  }
}

/** 更新频率，并在频率过高时自动切换到更快的扫描档位。 */
static void set_generator_frequency(uint16_t frequency_hz)
{
  if ((frequency_hz < FREQ_CONTROL_MIN) ||
      (frequency_hz > FREQ_CONTROL_MAX) ||
      (frequency_hz == generator_frequency_hz))
  {
    return;
  }

  generator_frequency_hz = frequency_hz; // 更新界面显示
  send_frequency_command(frequency_hz);  // 通知采集端

  /*
   * A square wave needs several samples per half cycle. Promote the
   * acquisition timebase automatically instead of displaying an aliased
   * low-frequency replica of a high-frequency generator signal.
   */
  if ((frequency_hz > FREQ_SWITCH_TO_100US_ABOVE) &&
      (timebase_index < 4U))
  {
    select_timebase(4U); // 100us/格
  }
  else if ((frequency_hz > FREQ_SWITCH_TO_500US_ABOVE) &&
           (timebase_index < 3U))
  {
    select_timebase(3U); // 500us/格
  }
  else if ((frequency_hz > FREQ_SWITCH_TO_1MS_ABOVE) &&
      (timebase_index < 2U))
  {
    select_timebase(2U); // 1ms/格
  }
  else if ((frequency_hz > FREQ_SWITCH_TO_5MS_ABOVE) &&
           (timebase_index < 1U))
  {
    select_timebase(1U); // 5ms/格
  }
  frame_ready = true;
}

/** 输出当前触发源、边沿和电平。 */
static void send_trigger_debug(void)
{
  char line[64];
  char sources[24] = "OFF";
  uint8_t position = 0U;

  if (trigger_any_source_enabled())
  {
    sources[0] = '\0';
    for (uint8_t index = TRIGGER_A1; index <= TRIGGER_D3; ++index)
    {
      if (!trigger_source_enabled((TriggerSource)index))
      {
        continue;
      }
      if (sources[0] != '\0')
      {
        strcat(sources, "+");
      }
      strcat(sources, trigger_source_name((TriggerSource)index));
    }
  }

  append_text(line, &position, sizeof(line), "TRIGGER,SOURCES=");
  append_text(line, &position, sizeof(line), sources);
  append_text(line, &position, sizeof(line), ",EDGE=");
  append_text(line, &position, sizeof(line),
              trigger_rising_edge ? "R" : "F");
  append_text(line, &position, sizeof(line), ",LEVEL_MV=");
  append_number(line, &position, sizeof(line), trigger_level_mv);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
}

/** 触发参数改变后发送调试信息并刷新界面。 */
static void trigger_settings_changed(void)
{
  reset_digital_scroll_offsets(); // 触发参数改变后重置三路滚动偏移
  send_trigger_debug();        // 向电脑报告新参数
  frame_ready = true;          // 标记需要刷新
  refresh_timebase_screen();   // 立即重绘界面
}

/** 设置触发源。 */
static void select_trigger_source(TriggerSource source)
{
  uint8_t old_mask = trigger_source_mask;
  uint8_t source_bit = trigger_source_bit(source);

  if (source == TRIGGER_OFF)
  {
    trigger_source_mask = 0U;
  }
  else if (source_bit != 0U)
  {
    trigger_source_mask ^= source_bit;
  }

  if (trigger_source_mask != old_mask)
  {
    trigger_settings_changed();
  }
}

/** 设置上升沿或下降沿触发。 */
static void select_trigger_edge(bool rising_edge)
{
  if (trigger_rising_edge != rising_edge)
  {
    trigger_rising_edge = rising_edge; // 保存边沿方向
    trigger_settings_changed();        // 统一刷新
  }
}

/** 在 0~3000mV 范围内调整触发电平。 */
static void adjust_trigger_level(int16_t delta_mv)
{
  int32_t level_mv = (int32_t)trigger_level_mv + delta_mv; // 新电平

  if (level_mv < 0)
  {
    level_mv = 0; // 最小 0mV
  }
  else if (level_mv > 3000)
  {
    level_mv = 3000; // 最大 3000mV
  }

  if ((uint16_t)level_mv != trigger_level_mv)
  {
    trigger_level_mv = (uint16_t)level_mv; // 保存限制后的值
    trigger_settings_changed();            // 刷新显示
  }
}

/** 输出扫描时间请求、确认或失败信息。 */
static void send_timebase_debug(const char *prefix, uint8_t index,
                                uint16_t interval)
{
  char line[48];
  uint8_t position = 0U;

  append_text(line, &position, sizeof(line), prefix);
  append_number(line, &position, sizeof(line), index);
  append_text(line, &position, sizeof(line), ",INTERVAL=");
  append_number(line, &position, sizeof(line), interval);
  append_text(line, &position, sizeof(line), "\r\n");
  BoardLink_WritePc((const uint8_t *)line, position);
}

/** 根据是否已有数据刷新等待界面或波形界面。 */
static void refresh_timebase_screen(void)
{
  if ((sample_count_ch1 > 0U) || (sample_count_ch2 > 0U))
  {
    render_received_frame();
  }
  else
  {
    render_waiting_screen();
  }
}

/**
 * 切换扫描时间并启动命令重发，直到采集端确认。
 */
static void select_timebase(uint8_t index)
{
  if (index >= TIMEBASE_COUNT)
  {
    return;
  }

  timebase_index = index;               // 先更新按钮高亮
  timebase_pending_index = index;       // 记录目标档位
  timebase_pending = true;              // 进入等待确认状态
  timebase_user_selected = true;        // 锁定用户选择
  timebase_failed = false;              // 清除旧失败标志
  timebase_retries_left = TOUCH_COMMAND_RETRIES - 1U; // 预留重发次数
  timebase_retry_at_ms = HAL_GetTick() + TOUCH_COMMAND_RETRY_MS; // 重发计时
  /* A timebase change is an explicit request to resume updating the trace. */
  waveform_paused = false; // 换档时自动恢复波形刷新
  send_timebase_command(index); // 发送控制包
  send_timebase_debug("TB REQ,IDX=", index, timebase_interval_us[index]);
  refresh_timebase_screen(); // 先显示新档位高亮
}

/** 处理换档命令的重发和超时。 */
static void poll_timebase_commands(void)
{
  uint32_t now;

  if (!timebase_pending)
  {
    if (timebase_user_selected && timebase_failed &&
        ((int32_t)(HAL_GetTick() - timebase_retry_at_ms) >= 0))
    {
      send_timebase_command(timebase_pending_index); // 失败后周期重试
      timebase_retry_at_ms = HAL_GetTick() + 1000U;  // 一秒后再次重试
    }
    return;
  }

  now = HAL_GetTick();
  if (timebase_retries_left == 0U)
  {
    if ((int32_t)(now - timebase_retry_at_ms) >= 0)
    {
      timebase_pending = false;         // 结束本轮等待
      timebase_failed = true;           // 标记换档失败
      timebase_failed_at_ms = now;      // 记录失败时刻
      timebase_retry_at_ms = now + 1000U; // 延后下一轮重试
      send_timebase_debug("TB FAIL,IDX=", timebase_pending_index,
                          timebase_interval_us[timebase_pending_index]);
      refresh_timebase_screen();
    }
    return;
  }

  if ((int32_t)(now - timebase_retry_at_ms) >= 0)
  {
    send_timebase_command(timebase_pending_index); // 重发目标档位
    --timebase_retries_left;                       // 消耗一次重试
    timebase_retry_at_ms = now + TOUCH_COMMAND_RETRY_MS; // 下次重发时间
  }
}

/* 三个物理/触摸按键的消抖状态，只供下方输入扫描函数使用。 */
static bool button_raw_pressed;           // PI11 原始状态
static uint32_t button_last_change_ms;    // PI11 上次变化时间
static bool touch_was_down;               // 上一次触摸状态
static uint32_t touch_last_change_ms;     // 上次触摸变化时间
static bool touch_ready;                  // 触摸控制器是否可用
static bool save_key_raw_pressed;         // D4 原始状态
static uint32_t save_key_last_change_ms;  // D4 上次变化时间

/** 扫描 PI11，并在按下边沿切换暂停状态。 */
static void poll_pause_button(void)
{
  bool pressed = // PI11 高电平表示按下
      HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_SET;
  uint32_t now = HAL_GetTick(); // 当前毫秒时间

  /* 板上 User 按键为高电平有效，35 ms 消抖。 */
  if ((pressed != button_raw_pressed) &&
      ((now - button_last_change_ms) >= BUTTON_DEBOUNCE_MS))
  {
    button_raw_pressed = pressed;    // 保存消抖后的状态
    button_last_change_ms = now;     // 更新状态变化时间

    if (pressed)
    {
      waveform_paused = !waveform_paused; // 暂停/继续
      frame_ready = false;                // 丢弃尚未刷新的旧帧
      refresh_timebase_screen();          // 立即显示 PAUSED
      BoardLink_WritePc(
          (const uint8_t *)(waveform_paused ? "PAUSE=1\r\n" : "PAUSE=0\r\n"), // 调试反馈
          (uint16_t)(sizeof("PAUSE=1\r\n") - 1U));
    }
  }
}

/** 扫描 D4，并在按下边沿保存快照。 */
static void poll_save_button(void)
{
  bool pressed = // D4 低电平表示按下
      HAL_GPIO_ReadPin(SAVE_KEY_GPIO_Port, SAVE_KEY_Pin) == GPIO_PIN_RESET;
  uint32_t now = HAL_GetTick(); // 当前毫秒时间

  if ((pressed != save_key_raw_pressed) &&
      ((now - save_key_last_change_ms) >= SAVE_KEY_DEBOUNCE_MS))
  {
    save_key_raw_pressed = pressed; // 保存消抖状态
    save_key_last_change_ms = now;  // 更新时间
    if (pressed)
    {
      (void)capture_snapshot(); // 冻结波形和屏幕
    }
  }
}

/** 切换指定通道的显示状态。 */
static void select_channel(uint8_t index)
{
  if (index >= CHANNEL_BUTTON_COUNT)
  {
    return;
  }

  channel_visible[index] = !channel_visible[index]; // 翻转通道开关
  refresh_timebase_screen();                        // 立即重绘按钮和波形
}

/**
 * 扫描触摸屏，依次处理触发菜单、滑块、扫描时间和通道按钮。
 */
static void poll_timebase_touch(void)
{
  uint16_t x;     // 触摸横坐标
  uint16_t y;     // 触摸纵坐标
  uint32_t now;   // 当前时间
  bool pressed;   // 当前是否触摸

  if (!touch_ready)
  {
    return;
  }

  pressed = Touchscreen_ReadPoint(&x, &y); // 读取有效触摸点
  now = HAL_GetTick();                     // 用于触摸消抖

  /* 频率滑块是连续控制，按住并滑动时每次都要更新。 */
  if (!trigger_menu_open && pressed &&
      (y >= FREQ_SLIDER_Y0) && (y <= FREQ_SLIDER_Y1) &&
      (x >= FREQ_TRACK_X0) && (x <= FREQ_TRACK_X1))
  {
    set_generator_frequency(frequency_from_slider(x)); // X 坐标换算频率
    touch_was_down = true;                            // 标记当前正在按住
    touch_last_change_ms = now;                       // 更新消抖时间
    return;
  }

  if ((pressed == touch_was_down) || // 状态未变化
      ((now - touch_last_change_ms) < TOUCH_DEBOUNCE_MS))
  {
    return;
  }

  touch_was_down = pressed;    // 保存新触摸状态
  touch_last_change_ms = now;  // 保存变化时间
  if (!pressed)
  {
    return;
  }

  /*
   * 触发设置面板打开时，触摸只交给面板处理。这样不会误触下面的
   * 扫描时间、通道显示或频率滑块，参数修改后面板仍保持打开。
   */
  if (trigger_menu_open)
  {
    if ((y >= 98U) && (y <= 120U)) // 第一行触发源
    {
      for (uint8_t index = 0U; index < 6U; ++index)
      {
        int x0 = 48 + (int)index * 62; // 当前源按钮左边界
        if ((x >= x0) && (x <= (x0 + 54)))
        {
          select_trigger_source((TriggerSource)index); // 设置触发源
          return;
        }
      }
    }

    if ((y >= 132U) && (y <= 154U)) // 第二行边沿选择
    {
      if ((x >= 48U) && (x <= 116U))
      {
        select_trigger_edge(true); // 上升沿
      }
      else if ((x >= 122U) && (x <= 190U))
      {
        select_trigger_edge(false); // 下降沿
      }
      return;
    }

    if ((y >= 166U) && (y <= 188U)) // 第三行电平调节
    {
      if ((x >= 48U) && (x <= 104U))
      {
        adjust_trigger_level(-500); // 粗减
      }
      else if ((x >= 110U) && (x <= 166U))
      {
        adjust_trigger_level(-100); // 细减
      }
      else if ((x >= 292U) && (x <= 348U))
      {
        adjust_trigger_level(100); // 细加
      }
      else if ((x >= 354U) && (x <= 410U))
      {
        adjust_trigger_level(500); // 粗加
      }
      return;
    }

    if ((y >= 198U) && (y <= 222U) &&
        (x >= 380U) && (x <= 438U))
    {
      trigger_menu_open = false; // 关闭面板
      refresh_timebase_screen(); // 恢复下层界面
    }
    return;
  }

  /* 底部 TRG 按钮：打开完整的触发参数设置面板。 */
  if ((y >= 242U) && (y <= 266U) && (x >= 68U) && (x <= 178U))
  {
    trigger_menu_open = true; // 打开触发设置
    refresh_timebase_screen();
    return;
  }

  /* 顶部五个扫描时间按钮。 */
  if ((y >= TOUCH_BUTTON_Y0) && (y <= TOUCH_BUTTON_Y1))
  {
    for (uint8_t index = 0U; index < TIMEBASE_COUNT; ++index)
    {
      int x0 = TOUCH_BUTTON_X0 +
               ((int)index * (TOUCH_BUTTON_WIDTH + TOUCH_BUTTON_GAP));
      if ((x >= x0) && (x < (x0 + TOUCH_BUTTON_WIDTH)))
      {
        select_timebase(index); // 切换到被点击档位
        break;
      }
    }
    return;
  }

  /* 第二排模拟/数字通道开关。 */
  if ((y >= CHANNEL_BUTTON_Y0) && (y <= CHANNEL_BUTTON_Y1))
  {
    for (uint8_t index = 0U; index < CHANNEL_BUTTON_COUNT; ++index)
    {
      int x0 = channel_button_x(index);
      if ((x >= x0) && (x < (x0 + CHANNEL_BUTTON_WIDTH)))
      {
        select_channel(index); // 翻转通道显示
        break;
      }
    }
  }
}

/**
 * 校验帧格式和 CRC，通过后拆分模拟量与数字量采样数据。
 */
static bool validate_and_store_frame(BoardLinkSource source) // 数据来源
{
  uint16_t count = (uint16_t)frame_buffer[6] | // 偏移 6~7：每通道点数
                   ((uint16_t)frame_buffer[7] << 8);
  uint16_t interval = (uint16_t)frame_buffer[8] | // 偏移 8~9：采样间隔
                      ((uint16_t)frame_buffer[9] << 8);
  uint16_t sequence = (uint16_t)frame_buffer[4] | // 偏移 4~5：帧序号
                      ((uint16_t)frame_buffer[5] << 8);
  uint8_t frame_type = frame_buffer[3];   // 帧类型
  uint8_t frame_flags = frame_buffer[10]; // 通道标志
  uint16_t payload_bytes;                 // 数据区长度
  uint8_t required_flags;                 // 当前帧必需标志
  uint8_t analog_channels;                // 模拟通道数
  uint8_t has_digital;                    // 是否带数字数据
  uint16_t received_crc;                  // 帧内 CRC
  uint16_t calculated_crc;                // 本地计算 CRC
  uint16_t index;                         // 采样点循环变量

  /* 第一步：确定帧类型对应的模拟通道数量。 */
  if (frame_type == FRAME_TYPE_CH1)
  {
    required_flags = FRAME_FLAG_CH1; // CH1 帧必须带 bit0
    analog_channels = 1U;            // 只有一路模拟量
  }
  else if (frame_type == FRAME_TYPE_CH2)
  {
    required_flags = FRAME_FLAG_CH2; // CH2 帧必须带 bit1
    analog_channels = 1U;
  }
  else if (frame_type == FRAME_TYPE_CH1_CH2)
  {
    required_flags = FRAME_FLAG_CH1_CH2; // 双模拟通道标志
    analog_channels = 2U;                // 两路模拟量
  }
  else
  {
    visual_state = RX_FORMAT_ERROR;
    send_frame_ack("FORMAT", sequence, count, interval, source);
    return false;
  }

  /* 第二步：检查通道标志和保留字节。 */
  has_digital = ((frame_flags & FRAME_FLAG_DIGITAL) != 0U) ? 1U : 0U; // 是否带数字位图
  if (((frame_flags & required_flags) != required_flags) ||
      ((frame_flags & (uint8_t)~(required_flags | FRAME_FLAG_DIGITAL)) != 0U) ||
      (frame_buffer[11] != 0U))
  {
    visual_state = RX_FORMAT_ERROR;
    send_frame_ack("FORMAT", sequence, count, interval, source);
    return false;
  }

  /* 第三步：检查点数、采样间隔，并计算数据区长度。 */
  payload_bytes = (uint16_t)(
      count * ((analog_channels * 2U) + has_digital)); // 数据区字节数
  if ((count == 0U) || (count > APP_FRAME_MAX_SAMPLES) ||
      ((interval != 2U) && (interval != 10U) && (interval != 20U) &&
       (interval != 100U) && (interval != 200U)))
  {
    visual_state = RX_FORMAT_ERROR;
    send_frame_ack("FORMAT", sequence, count, interval, source);
    return false;
  }

  /* 第四步：比较发送端 CRC 和本地重新计算的 CRC。 */
  received_crc = (uint16_t)frame_buffer[12U + payload_bytes] |
                 ((uint16_t)frame_buffer[13U + payload_bytes] << 8); // 帧尾 CRC
  calculated_crc = crc16_ccitt(frame_buffer, 12U + payload_bytes);       // 本地 CRC
  if (received_crc != calculated_crc)
  {
    ++crc_error_count;
    visual_state = RX_CRC_ERROR;
    send_frame_ack("CRC", sequence, count, interval, source);
    return false;
  }

  /* 第五步：按每个采样点的字节布局拆分通道数据。 */
  for (index = 0U; index < count; ++index)
  {
    uint16_t step = (uint16_t)((analog_channels * 2U) + has_digital); // 每点字节数
    uint16_t sample_offset = (uint16_t)(12U + (step * index));         // 当前点偏移

    if ((frame_type == FRAME_TYPE_CH1) ||
        (frame_type == FRAME_TYPE_CH1_CH2))
    {
      uint16_t value = (uint16_t)frame_buffer[sample_offset] |
                       ((uint16_t)frame_buffer[sample_offset + 1U] << 8);
      samples_ch1[index] = (value > 3000U) ? 3000U : value; // 限制在 0~3000mV
    }

    if ((frame_type == FRAME_TYPE_CH2) ||
        (frame_type == FRAME_TYPE_CH1_CH2))
    {
      uint16_t channel_offset =
          (frame_type == FRAME_TYPE_CH1_CH2) ? 2U : 0U;
      uint16_t value = (uint16_t)frame_buffer[sample_offset + channel_offset] |
                       ((uint16_t)frame_buffer[sample_offset +
                                               channel_offset + 1U] << 8);
      samples_ch2[index] = (value > 3000U) ? 3000U : value; // 限制在 0~3000mV
    }

    if (has_digital != 0U)
    {
      samples_digital[index] =
          frame_buffer[sample_offset + (analog_channels * 2U)] & 0x07U; // D1~D3
    }
  }

  /* 第六步：更新当前帧序号和各通道有效点数。 */
  frame_sequence = sequence;
  if ((frame_type == FRAME_TYPE_CH1) ||
      (frame_type == FRAME_TYPE_CH1_CH2))
  {
    sample_count_ch1 = count;
  }
  if ((frame_type == FRAME_TYPE_CH2) ||
      (frame_type == FRAME_TYPE_CH1_CH2))
  {
    sample_count_ch2 = count;
  }
  sample_count_digital = (has_digital != 0U) ? count : 0U; // 无数字数据时清零
  sample_interval_us = interval;                            // 更新显示采样间隔

  /* 第七步：用新数据重新计算峰值、频率和占空比。 */
  if ((frame_type == FRAME_TYPE_CH1) ||
      (frame_type == FRAME_TYPE_CH1_CH2))
  {
    measure_analog_channel(samples_ch1, sample_count_ch1, interval,
                           &analog_measurements[0]);
  }
  if ((frame_type == FRAME_TYPE_CH2) ||
      (frame_type == FRAME_TYPE_CH1_CH2))
  {
    measure_analog_channel(samples_ch2, sample_count_ch2, interval,
                           &analog_measurements[1]);
  }
  if (has_digital != 0U)
  {
    measure_digital_channel(0x01U, interval, &digital_measurements[0]);
    measure_digital_channel(0x02U, interval, &digital_measurements[1]);
    measure_digital_channel(0x04U, interval, &digital_measurements[2]);
  }
  /* 第八步：同步用户选择的扫描档位和采集端实际返回的采样间隔。 */
  if (timebase_user_selected)
  {
    if (interval == timebase_interval_us[timebase_pending_index])
    {
      timebase_index = timebase_pending_index; // 采集端确认新档位
      timebase_pending = false;                 // 停止重发
      timebase_failed = false;                  // 清除失败状态
      timebase_user_selected = false;           // 允许跟随数据帧
      send_timebase_debug("TB OK,IDX=", timebase_index, interval);
    }
    else if (!timebase_pending)
    {
      /*
       * Keep the user's selected button locked while the acquisition board
       * is still sending the old interval. Otherwise every old frame makes
       * the button jump back to 10MS before the switch is confirmed.
       */
      timebase_failed = true; // 旧档位仍到达，标记等待确认
    }
  }
  else
  {
    switch (interval)
    {
      case 2U:   timebase_index = 4U; break;
      case 10U:  timebase_index = 3U; break;
      case 20U:  timebase_index = 2U; break;
      case 100U: timebase_index = 1U; break;
      default:   timebase_index = 0U; break;
    }
  }

  /* 第九步：整帧通过，更新成功状态并通知主循环刷新。 */
  ++valid_frame_count;        // 有效帧计数加一
  visual_state = RX_FRAME_OK; // 状态栏显示接收正常
  frame_ready = true;         // 通知主循环刷新
  send_frame_ack("OK", frame_sequence, count, interval, source);
  send_measurement_debug();
  return true;
}

/** 解析 `GF=频率` 电脑命令。 */
static bool parse_frequency_command(uint8_t length, uint16_t *frequency)
{
  uint32_t value = 0U;

  if ((frequency == NULL) || (length < 4U) ||
      (pc_command_buffer[0] != 'G') ||
      (pc_command_buffer[1] != 'F') ||
      (pc_command_buffer[2] != '='))
  {
    return false;
  }

  for (uint8_t index = 3U; index < length; ++index)
  {
    char character = pc_command_buffer[index];
    if ((character < '0') || (character > '9'))
    {
      return false;
    }
    value = (value * 10U) + (uint32_t)(character - '0');
  }

  if ((value < FREQ_CONTROL_MIN) || (value > FREQ_CONTROL_MAX))
  {
    return false;
  }
  *frequency = (uint16_t)value;
  return true;
}

/** 解析 `TL=毫伏` 触发电平命令。 */
static bool parse_trigger_level_command(uint8_t length, uint16_t *level)
{
  uint32_t value = 0U;

  if ((level == NULL) || (length < 4U) ||
      (pc_command_buffer[0] != 'T') ||
      (pc_command_buffer[1] != 'L') ||
      (pc_command_buffer[2] != '='))
  {
    return false;
  }

  for (uint8_t index = 3U; index < length; ++index)
  {
    char character = pc_command_buffer[index];
    if ((character < '0') || (character > '9'))
    {
      return false;
    }
    value = (value * 10U) + (uint32_t)(character - '0');
  }
  if (value > 3000U)
  {
    return false;
  }
  *level = (uint16_t)value;
  return true;
}

/**
 * 串口字节状态机，区分电脑文本命令、板间文本命令和二进制波形帧。
 */
static void parser_push(uint8_t byte, BoardLinkSource source) // 当前字节/来源
{
  /**
   * 电脑发来的都是可读文本命令，因此使用独立缓冲区和解析分支，
   * 不会进入 A5 5A 二进制帧重组状态机。
   */
  if (source == BOARD_LINK_SOURCE_PC)
  {
    if ((byte == '\r') || (byte == '\n')) // 一整条文本命令结束
    {
      if (pc_command_length > 0U)
      {
        pc_command_buffer[pc_command_length] = '\0'; // 补字符串结束符
        if ((pc_command_length == 4U) &&
            (pc_command_buffer[0] == 'T') &&
            (pc_command_buffer[1] == 'B') &&
            (pc_command_buffer[2] == '=') &&
            (pc_command_buffer[3] >= '0') &&
            (pc_command_buffer[3] <= '4'))
        {
          select_timebase((uint8_t)(pc_command_buffer[3] - '0'));
        }
        else if ((pc_command_length == 4U) &&
                 (memcmp(pc_command_buffer, "SAVE", 4U) == 0))
        {
          (void)capture_snapshot();
        }
        else if ((pc_command_length == 3U) &&
                 (memcmp(pc_command_buffer, "CSV", 3U) == 0))
        {
          send_csv_snapshot();
        }
        else if ((pc_command_length == 3U) &&
                 (memcmp(pc_command_buffer, "BMP", 3U) == 0))
        {
          send_bmp_snapshot();
        }
        else if (strcmp(pc_command_buffer, "TS=OFF") == 0)
        {
          select_trigger_source(TRIGGER_OFF);
        }
        else if (strcmp(pc_command_buffer, "TS=A1") == 0)
        {
          select_trigger_source(TRIGGER_A1);
        }
        else if (strcmp(pc_command_buffer, "TS=A2") == 0)
        {
          select_trigger_source(TRIGGER_A2);
        }
        else if (strcmp(pc_command_buffer, "TS=D1") == 0)
        {
          select_trigger_source(TRIGGER_D1);
        }
        else if (strcmp(pc_command_buffer, "TS=D2") == 0)
        {
          select_trigger_source(TRIGGER_D2);
        }
        else if (strcmp(pc_command_buffer, "TS=D3") == 0)
        {
          select_trigger_source(TRIGGER_D3);
        }
        else if (strcmp(pc_command_buffer, "TE=R") == 0)
        {
          trigger_rising_edge = true;
          send_trigger_debug();
          frame_ready = true;
        }
        else if (strcmp(pc_command_buffer, "TE=F") == 0)
        {
          trigger_rising_edge = false;
          send_trigger_debug();
          frame_ready = true;
        }
        else
        {
          uint16_t value;
          if (parse_trigger_level_command(pc_command_length, &value))
          {
            trigger_level_mv = value;
            send_trigger_debug();
            frame_ready = true;
          }
          else if (parse_frequency_command(pc_command_length, &value))
          {
            set_generator_frequency(value);
          }
        }
        pc_command_length = 0U;
      }
    }
    else if ((byte >= 0x20U) && (byte <= 0x7EU) &&
             (pc_command_length < PC_COMMAND_MAX_LENGTH))
    {
      pc_command_buffer[pc_command_length++] = (char)byte; // 暂存可打印字符
    }
    else
    {
      pc_command_length = 0U;
    }
    return;
  }

  /**
   * 采集板链路同时兼容文本调试命令和二进制波形帧。
   * 下面先判断一帧是否尚未开始，再根据字节寻找 A5 5A 同步头。
   */
  if (frame_length == 0U)
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (pc_command_length > 0U)
      {
        pc_command_buffer[pc_command_length] = '\0';
        if ((pc_command_length == 4U) &&
            (pc_command_buffer[0] == 'T') &&
            (pc_command_buffer[1] == 'B') &&
            (pc_command_buffer[2] == '=') &&
            (pc_command_buffer[3] >= '0') &&
            (pc_command_buffer[3] <= '4'))
        {
          select_timebase((uint8_t)(pc_command_buffer[3] - '0'));
        }
        pc_command_length = 0U;
      }
      return;
    }

    if ((byte >= 0x20U) && (byte <= 0x7EU) &&
        (pc_command_length < PC_COMMAND_MAX_LENGTH))
    {
      pc_command_buffer[pc_command_length++] = (char)byte;
      return;
    }

    /* 收到二进制帧头或非法控制字节时退出文本命令状态。 */
    pc_command_length = 0U;
  }

  /* 状态机从同步头 A5 5A 开始，坏数据不会导致整帧解析越界。 */
  if (frame_length == 0U)
  {
    if (byte == FRAME_SYNC_0) frame_buffer[frame_length++] = byte; // 等待 A5
    return;
  }
  if (frame_length == 1U)
  {
    if (byte == FRAME_SYNC_1)
    {
      frame_buffer[frame_length++] = byte; // 收到 5A，帧头开始
    }
    else if (byte != FRAME_SYNC_0)
    {
      frame_length = 0U; // 第二个同步字节错误，重新开始
    }
    return;
  }

  frame_buffer[frame_length++] = byte; // 收到帧头或数据字节
  if (frame_length == APP_FRAME_HEADER_SIZE)
  {
    uint16_t count = (uint16_t)frame_buffer[6] |
      ((uint16_t)frame_buffer[7] << 8); // 每通道点数，小端
    uint8_t type = frame_buffer[3];     // 帧类型
    uint8_t flags = frame_buffer[10];   // 通道标志
    bool valid_type = (type == FRAME_TYPE_CH1) || // 帧类型是否支持
                      (type == FRAME_TYPE_CH2) ||
                      (type == FRAME_TYPE_CH1_CH2);
    bool valid_flags = // 标志位必须合法且包含模拟通道
        ((flags & (uint8_t)~FRAME_FLAG_ALL) == 0U) &&
        ((flags & FRAME_FLAG_CH1_CH2) != 0U);
    if ((frame_buffer[2] == FRAME_VERSION) && valid_type &&
        valid_flags && (count != 0U) && (count <= APP_FRAME_MAX_SAMPLES))
    {
      uint16_t channel_words = (type == FRAME_TYPE_CH1_CH2) ? 4U : 2U; // 每点模拟字节
      if ((flags & FRAME_FLAG_DIGITAL) != 0U)
      {
        channel_words++;
      }
      expected_frame_length = (uint16_t)(
          APP_FRAME_HEADER_SIZE + (channel_words * count) + 2U); // 帧总长度
      frame_header_valid = true; // 帧头参数合法
    }
    else
    {
      visual_state = RX_FORMAT_ERROR;
      reset_parser();
      if (byte == FRAME_SYNC_0) frame_buffer[frame_length++] = byte; // 当前字节可能是新帧头
    }
  }

  if (frame_header_valid && (frame_length == expected_frame_length))
  {
    (void)validate_and_store_frame(source); // 长度已齐，校验整帧
    reset_parser();                         // 准备下一帧
  }
}

/* ============================ 7. 硬件初始化 ============================ */
/*
 * 对板载 SDRAM 执行上电初始化命令序列。
 * 这个函数只执行一次，必须在应用读取 0xC0000000 帧缓冲区之前完成。
 */
static void SDRAM_DeviceInit(void)
{
  FMC_SDRAM_CommandTypeDef command = {0};
  uint32_t mode_register;

  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  command.AutoRefreshNumber = 1U;
  command.ModeRegisterDefinition = 0U;

  /* SDRAM 上电初始化必须严格按照 JEDEC 命令顺序执行。 */
  /* 1. 打开 SDRAM 时钟并等待至少 100 us。 */
  command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, 0xFFFFU) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(1U);

  /* 2. 所有 Bank 预充电。 */
  command.CommandMode = FMC_SDRAM_CMD_PALL;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, 0xFFFFU) != HAL_OK)
  {
    Error_Handler();
  }

  /* 3. 连续执行 8 次自动刷新。 */
  command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  command.AutoRefreshNumber = 8U;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, 0xFFFFU) != HAL_OK)
  {
    Error_Handler();
  }

  /* 4. 写模式寄存器：突发长度 1、顺序突发、CAS=2、单次写。 */
  mode_register = 0x00000220U;
  command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  command.AutoRefreshNumber = 1U;
  command.ModeRegisterDefinition = mode_register;
  if (HAL_SDRAM_SendCommand(&hsdram1, &command, 0xFFFFU) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 5. FMC 时钟为 108 MHz，SDRAM 有 4096 行。
   * 刷新计数 = ceil(64 ms * 108 MHz / 4096) - 20 = 1668。
   */
  if (HAL_SDRAM_ProgramRefreshRate(&hsdram1, 1668U) != HAL_OK)
  {
    Error_Handler();
  }
}

/* SDRAM 中存放的是 CPU 与 LTDC 共享的图像内存，因此设置成不可缓存。 */
static void configure_framebuffer_mpu(void)
{
  MPU_Region_InitTypeDef mpu = {0};

  /*
   * 外部 SDRAM 同时被 CPU 写入、被 LTDC 读取。
   * 设为可访问、Shareable、不可缓存，避免缓存中的旧帧被显示。
   */
  HAL_MPU_Disable();
  mpu.Enable = MPU_REGION_ENABLE;
  mpu.Number = MPU_REGION_NUMBER0;
  mpu.BaseAddress = LCD_FB0_ADDRESS;
  mpu.Size = MPU_REGION_SIZE_16MB;
  mpu.SubRegionDisable = 0x00U;
  mpu.TypeExtField = MPU_TEX_LEVEL1;
  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  mpu.IsShareable = MPU_ACCESS_SHAREABLE;
  mpu.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  mpu.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/** 初始化 SDRAM、LCD、触摸、串口和接收端运行状态。 */
void App_ReceiverInit(void)
{
  configure_framebuffer_mpu(); // 配置 SDRAM 的 MPU 属性
  SDRAM_DeviceInit();          // 执行 SDRAM 上电命令序列

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET); // LCD 显示使能
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);  // LCD 背光使能
  Touchscreen_Init();                                  // 初始化 I2C 触摸
  touch_ready = Touchscreen_IsReady();                 // 记录触摸是否可用

  /* 启动时清空两块显示缓冲区。 */
  memset((void *)framebuffers[0], 0,
         LCD_FRAME_PIXELS * sizeof(uint16_t));
  memset((void *)framebuffers[1], 0,
         LCD_FRAME_PIXELS * sizeof(uint16_t));

  /* 恢复扫描档位、暂停和数字滚动状态。 */
  active_framebuffer = 0U;
  timebase_index = 0U;
  timebase_pending = false;
  timebase_pending_index = 0U;
  timebase_retries_left = 0U;
  timebase_failed = false;
  timebase_failed_at_ms = 0U;
  timebase_user_selected = false;
  waveform_paused = false;
  reset_digital_scroll_offsets();
  /* 上电时默认打开全部通道。 */
  for (uint8_t index = 0U; index < CHANNEL_BUTTON_COUNT; ++index)
  {
    channel_visible[index] = true;
  }
  /* 记录按键和触摸的初始电平，防止第一次扫描被误判为按下。 */
  pc_command_length = 0U;
  touch_was_down = false;
  touch_last_change_ms = HAL_GetTick();
  button_raw_pressed =
      HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_SET;
  button_last_change_ms = HAL_GetTick();
  save_key_raw_pressed =
      HAL_GPIO_ReadPin(SAVE_KEY_GPIO_Port, SAVE_KEY_Pin) == GPIO_PIN_RESET;
  save_key_last_change_ms = HAL_GetTick();
  /* 上电时没有可导出的快照。 */
  snapshot_ready = false;
  save_status_until_ms = 0U;
  save_status_ok = false;
  /* 绘制一次静态背景并保存副本，之后每帧直接复制。 */
  draw_grid(framebuffers[0]);
  memcpy(background_framebuffer, framebuffers[0],
         LCD_FRAME_PIXELS * sizeof(uint16_t));
  present_frame(framebuffers[0]);
  render_waiting_screen();

  /* 串口启动后再请求默认 1ms 档位。 */
  BoardLink_Init();    // 启动串口接收
  select_timebase(2U); // 默认每格 1ms
  /* 向电脑报告接收端和触摸控制器是否就绪。 */
  {
    char banner[40];
    uint8_t position = 0U;

    append_text(banner, &position, sizeof(banner), "R2 READY,VER=1,TOUCH=");
    if (touch_ready)
    {
      append_number(banner, &position, sizeof(banner),
                    Touchscreen_GetChipId());
    }
    else
    {
      append_text(banner, &position, sizeof(banner), "ERR");
    }
    append_text(banner, &position, sizeof(banner), "\r\n");
    BoardLink_WritePc((const uint8_t *)banner, position);
  }
  BoardLink_WriteBoard((const uint8_t *)"LINK READY,ROLE=RX\r\n", 20U);
}

/** 接收端主循环：解析数据、扫描输入并刷新波形。 */
void App_ReceiverLoop(void)
{
  uint16_t processed = 0U;
  uint8_t value;

  /**
   * 第一步：优先处理电脑命令。
   * 板间波形数据量很大，如果全部先处理板间数据，电脑命令可能长时间得不到响应。
   */
  while ((processed < RX_BYTES_PER_LOOP) &&
         BoardLink_ReadPcByte(&value))
  {
    parser_push(value, BOARD_LINK_SOURCE_PC); // 解析电脑文本命令
    ++processed;                              // 统计本轮字节数
  }

  /**
   * 第二步：处理采集板通过 UART7 发来的波形数据字节。
   */
  while ((processed < RX_BYTES_PER_LOOP) &&
         BoardLink_ReadBoardByte(&value))
  {
    parser_push(value, BOARD_LINK_SOURCE_BOARD); // 解析板间二进制数据
    ++processed;                                 // 统计本轮字节数
  }

  /**
   * 第三步：检查 UART 错误、按键、触摸操作和未确认的换档命令。
   */
  BoardLink_Service();      // 检查串口状态
  poll_pause_button();      // 扫描 PI11
  poll_save_button();       // 扫描 D4
  poll_timebase_touch();    // 扫描触摸屏
  poll_timebase_commands(); // 处理换档重发

  /**
   * 第四步：收到完整新帧且没有暂停时，绘制新一帧波形。
   */
  if (frame_ready)
  {
    frame_ready = false; // 清除刷新请求
    if (!waveform_paused)
    {
      render_received_frame(); // 绘制新帧
    }
  }

  if (processed == 0U)
  {
    HAL_Delay(1U); // 无数据时降低 CPU 空转
  }
}

/* 以下三个函数只返回调试统计值，方便断点观察接收质量。 */
uint32_t App_GetValidFrameCount(void)
{
  return valid_frame_count;
}

uint32_t App_GetCrcErrorCount(void)
{
  return crc_error_count;
}

uint32_t App_GetRingDropCount(void)
{
  return BoardLink_GetBoardDropCount() + BoardLink_GetPcDropCount();
}
