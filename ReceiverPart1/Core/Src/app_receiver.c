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

#define PLOT_LEFT 42
#define PLOT_RIGHT 474
#define PLOT_TOP 48
#define PLOT_BOTTOM 236
#define PLOT_WIDTH ((PLOT_RIGHT - PLOT_LEFT) + 1)
#define PLOT_HEIGHT ((PLOT_BOTTOM - PLOT_TOP) + 1)

/* ============================ 2. 上下行协议 ============================ */
#define FRAME_SYNC_0 0xA5U
#define FRAME_SYNC_1 0x5AU
#define FRAME_VERSION 0x01U
#define FRAME_TYPE_CH1 0x10U
#define FRAME_TYPE_CH2 0x20U
#define FRAME_TYPE_CH1_CH2 0x30U
#define FRAME_FLAG_CH1 0x01U
#define FRAME_FLAG_CH2 0x02U
#define FRAME_FLAG_CH1_CH2 (FRAME_FLAG_CH1 | FRAME_FLAG_CH2)

#define RX_BYTES_PER_LOOP 1024U
#define BUTTON_DEBOUNCE_MS 35U
#define TOUCH_DEBOUNCE_MS 120U
#define TOUCH_COMMAND_RETRY_MS 150U
#define TOUCH_COMMAND_RETRIES 3U
#define PC_COMMAND_MAX_LENGTH 15U

#define CONTROL_SYNC_0 0xC5U
#define CONTROL_SYNC_1 0x5CU
#define CONTROL_VERSION 0x01U
#define CONTROL_CMD_TIMEBASE 0x01U

/* 顶部三个直接档位按钮。 */
#define TOUCH_BUTTON_Y0 4
#define TOUCH_BUTTON_Y1 35
#define TOUCH_BUTTON_WIDTH 66
#define TOUCH_BUTTON_10MS_X0 276
#define TOUCH_BUTTON_5MS_X0 344
#define TOUCH_BUTTON_1MS_X0 412

/* ============================ 3. 运行状态 ============================ */
/* 两个帧缓冲区都位于外部 SDRAM，元素类型是 RGB565 的 16 位颜色。 */
static uint16_t *const framebuffers[2] = {
    (uint16_t *)LCD_FB0_ADDRESS,
    (uint16_t *)LCD_FB1_ADDRESS,
};

/*
 * 保存当前正在拼接的一帧。
 * frame_length        当前已经收到了多少字节；
 * expected_frame_length 根据帧头计算出的完整长度；
 * frame_header_valid  帧头检查通过的标志。
 */
static uint8_t frame_buffer[APP_FRAME_MAX_SIZE];
static uint16_t frame_length;
static uint16_t expected_frame_length;
static bool frame_header_valid;
static char pc_command_buffer[PC_COMMAND_MAX_LENGTH + 1U];
static uint8_t pc_command_length;

/*
 * 校验成功的采样值按通道分开保存。
 * 电压单位为毫伏，因此 1500 实际表示 1.500 V。
 */
static uint16_t samples_ch1[APP_FRAME_MAX_SAMPLES];
static uint16_t samples_ch2[APP_FRAME_MAX_SAMPLES];
static uint16_t sample_count_ch1;
static uint16_t sample_count_ch2;
static uint16_t sample_interval_us;
static uint16_t frame_sequence;
static volatile bool frame_ready;

/* 调试和运行统计变量：接收成功数、CRC 错误数、缓冲区溢出数。 */
static volatile uint32_t valid_frame_count;
static volatile uint32_t crc_error_count;
static uint8_t active_framebuffer;
static bool button_raw_pressed;
static uint32_t button_last_change_ms;
static bool waveform_paused;
static bool touch_was_down;
static uint32_t touch_last_change_ms;
static bool touch_ready;

/*
 * 数组下标就是 timebase_index。
 * 50 点/格时，200 us × 50 = 10 ms，其余两档同理。
 */
static const uint16_t timebase_interval_us[] = {200U, 100U, 20U};
static const char *const timebase_labels[] = {"10MS", "5MS", "1MS"};
static uint8_t timebase_index;
static bool timebase_pending;
static uint8_t timebase_pending_index;
static uint8_t timebase_retries_left;
static uint32_t timebase_retry_at_ms;
static bool timebase_failed;
static uint32_t timebase_failed_at_ms;

typedef enum
{
  RX_WAITING = 0,
  RX_FRAME_OK,
  RX_CRC_ERROR,
  RX_FORMAT_ERROR
} RxVisualState;

static RxVisualState visual_state = RX_WAITING;

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
  return (uint16_t)(((uint16_t)(red & 0xF8U) << 8) |
                    ((uint16_t)(green & 0xFCU) << 3) |
                    ((uint16_t)blue >> 3));
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
      (y >= 0) && (y < (int)LCD_HEIGHT))
  {
    framebuffer[((uint32_t)y * LCD_WIDTH) + (uint32_t)x] = color;
  }
}

/* 用同一种颜色填充矩形区域，x0/y0 为左上角，x1/y1 为右下角。 */
static void fill_rect(uint16_t *framebuffer, int x0, int y0, int x1, int y1,
                      uint16_t color)
{
  int x;
  int y;

  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 >= (int)LCD_WIDTH) x1 = (int)LCD_WIDTH - 1;
  if (y1 >= (int)LCD_HEIGHT) y1 = (int)LCD_HEIGHT - 1;
  if ((x0 > x1) || (y0 > y1)) return;

  for (y = y0; y <= y1; ++y)
  {
    uint16_t *row = framebuffer + ((uint32_t)y * LCD_WIDTH);
    for (x = x0; x <= x1; ++x)
    {
      row[x] = color;
    }
  }
}

/* 画一条水平线，内部复用矩形填充。 */
static void line_h(uint16_t *framebuffer, int x0, int x1, int y,
                   uint16_t color)
{
  if (x0 > x1)
  {
    int temp = x0;
    x0 = x1;
    x1 = temp;
  }
  fill_rect(framebuffer, x0, y, x1, y, color);
}

/* 画一条垂直线，内部复用矩形填充。 */
static void line_v(uint16_t *framebuffer, int x, int y0, int y1,
                   uint16_t color)
{
  if (y0 > y1)
  {
    int temp = y0;
    y0 = y1;
    y1 = temp;
  }
  fill_rect(framebuffer, x, y0, x, y1, color);
}

/*
 * 返回一个字符的 5×7 点阵数据。
 * 每个字符由 5 列组成，每列 7 位有效，位值为 1 的位置会点亮。
 */
static const uint8_t *glyph_for(char character)
{
  static const uint8_t space[5] = {0, 0, 0, 0, 0};
  static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
  static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
  static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
  static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};

  if ((character >= '0') && (character <= '9'))
  {
    return font_5x7[character - '0'];
  }
  if ((character >= 'A') && (character <= 'Z'))
  {
    return font_5x7[10 + (character - 'A')];
  }
  if (character == '.') return dot;
  if (character == ':') return colon;
  if (character == '-') return dash;
  if (character == '/') return slash;
  return space;
}

/*
 * 在 framebuffer 上绘制字符串。
 * scale=1 为正常大小，scale=2 时宽高都放大两倍。
 * 返回值是字符串末尾的 x 坐标，便于继续拼接下一段文字。
 */
static int draw_text(uint16_t *framebuffer, int x, int y, const char *text,
                     uint16_t color, uint8_t scale)
{
  int cursor = x;

  while (*text != '\0')
  {
    const uint8_t *glyph = glyph_for(*text++);
    uint8_t column;
    uint8_t row;

    for (column = 0; column < 5U; ++column)
    {
      for (row = 0; row < 7U; ++row)
      {
        if (((glyph[column] >> row) & 0x01U) != 0U)
        {
          fill_rect(framebuffer,
                    cursor + ((int)column * scale),
                    y + ((int)row * scale),
                    cursor + ((int)column * scale) + scale - 1,
                    y + ((int)row * scale) + scale - 1,
                    color);
        }
      }
    }
    cursor += 6 * scale;
  }
  return cursor;
}

/* 把整数毫伏值格式化成 0.000V~3.000V 的屏幕字符串。 */
static void format_voltage(uint16_t millivolts, char *output)
{
  uint16_t whole = millivolts / 1000U;
  uint16_t fraction = millivolts % 1000U;

  output[0] = (char)('0' + whole);
  output[1] = '.';
  output[2] = (char)('0' + (fraction / 100U));
  output[3] = (char)('0' + ((fraction / 10U) % 10U));
  output[4] = (char)('0' + (fraction % 10U));
  output[5] = 'V';
  output[6] = '\0';
}

/* 将无符号整数转换为十进制字符串，不依赖 printf，减少代码体积。 */
static void format_uint(char *output, uint32_t value)
{
  char reverse[11];
  uint8_t count = 0;
  uint8_t index = 0;

  if (value == 0U)
  {
    output[0] = '0';
    output[1] = '\0';
    return;
  }

  while ((value > 0U) && (count < sizeof(reverse)))
  {
    reverse[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while (count > 0U)
  {
    output[index++] = reverse[--count];
  }
  output[index] = '\0';
}

/* ============================ 5. 屏幕布局与波形渲染 ============================ */
/*
 * 绘制静态背景：深色面板、10 个时间格、4 个电压分区、刻度和按键提示。
 * 每次刷新波形时先重建背景，避免旧波形残留在屏幕上。
 */
static void draw_timebase_buttons(uint16_t *framebuffer)
{
  static const int button_x[] = {
      TOUCH_BUTTON_10MS_X0,
      TOUCH_BUTTON_5MS_X0,
      TOUCH_BUTTON_1MS_X0,
  };
  uint8_t index;

  for (index = 0U; index < 3U; ++index)
  {
    int x0 = button_x[index];
    int x1 = x0 + TOUCH_BUTTON_WIDTH - 1;
    uint16_t fill = (index == timebase_index) ? COLOR_CH1 : COLOR_PANEL;
    uint16_t text = (index == timebase_index) ? COLOR_BACKGROUND : COLOR_TEXT;

    fill_rect(framebuffer, x0, TOUCH_BUTTON_Y0, x1, TOUCH_BUTTON_Y1, fill);
    draw_text(framebuffer, x0 + 24, 15, timebase_labels[index], text, 1);
  }
}

static void draw_grid(uint16_t *framebuffer)
{
  uint8_t i;

  fill_rect(framebuffer, 0, 0, (int)LCD_WIDTH - 1,
            (int)LCD_HEIGHT - 1, COLOR_BACKGROUND);
  fill_rect(framebuffer, 0, 0, (int)LCD_WIDTH - 1, 38, COLOR_PANEL);
  fill_rect(framebuffer, (int)PLOT_LEFT, (int)PLOT_TOP,
            (int)PLOT_RIGHT, (int)PLOT_BOTTOM, COLOR_PLOT);

  /* 10 个水平时间格。 */
  for (i = 0; i <= 10U; ++i)
  {
    int x = PLOT_LEFT + ((int)i * (PLOT_WIDTH - 1)) / 10;
    line_v(framebuffer, x, PLOT_TOP, PLOT_BOTTOM,
           (i == 0U || i == 10U) ? COLOR_GRID_MAJOR : COLOR_GRID);
  }

  /* 四等分纵向电压刻度，对应 3.0 / 1.5 / 0 V。 */
  for (i = 0; i <= 4U; ++i)
  {
    int y = PLOT_TOP + ((int)i * (PLOT_HEIGHT - 1)) / 4;
    line_h(framebuffer, PLOT_LEFT, PLOT_RIGHT, y,
           (i == 0U || i == 4U) ? COLOR_GRID_MAJOR : COLOR_GRID);
  }

  draw_text(framebuffer, 4, 52, "3.0", COLOR_MUTED, 1);
  draw_text(framebuffer, 4, 142, "1.5", COLOR_MUTED, 1);
  draw_text(framebuffer, 4, 226, "0.0", COLOR_MUTED, 1);
  draw_text(framebuffer, 4, 4, "T/DIV", COLOR_MUTED, 1);
  draw_text(framebuffer, 8, 248,
            touch_ready ? "TOUCH OK" : "TOUCH ERR",
            touch_ready ? COLOR_CH1 : COLOR_ERROR, 1);
  draw_timebase_buttons(framebuffer);
}

/*
 * 将一个通道的采样值绘制为折线。
 * data  指向该通道的有效采样数组；
 * count 是有效点数；
 * color 决定 CH1 为亮青色、CH2 为琥珀色。
 */
static void draw_waveform(uint16_t *framebuffer, const uint16_t *data,
                          uint16_t count, uint16_t color)
{
  int x;
  int previous_center = PLOT_BOTTOM;
  bool have_previous = false;

  if ((data == NULL) || (count == 0U))
  {
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
      if (data[index] < minimum) minimum = data[index];
      if (data[index] > maximum) maximum = data[index];
    }

    if (maximum > 3000U) maximum = 3000U;
    if (minimum > 3000U) minimum = 3000U;

    y_top = PLOT_BOTTOM -
            ((int)maximum * (PLOT_HEIGHT - 1)) / 3000;
    y_bottom = PLOT_BOTTOM -
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
      line_v(framebuffer, x, previous_center,
             (y_top + y_bottom) / 2, color);
    }
    previous_center = (y_top + y_bottom) / 2;
    have_previous = true;
  }
}

/* 在顶部绘制通道名称与最后一个采样值，例如 CH1 POT 1.532V。 */
static void draw_channel_reading(uint16_t *framebuffer, int label_x,
                                 const char *label, uint16_t color,
                                 const uint16_t *data, uint16_t count)
{
  char voltage[8];

  draw_text(framebuffer, label_x, 13, label, color, 1);
  if ((data != NULL) && (count > 0U))
  {
    format_voltage(data[count - 1U], voltage);
    draw_text(framebuffer, label_x + CHANNEL_VALUE_OFFSET, 13,
              voltage, COLOR_TEXT, 1);
  }
  else
  {
    draw_text(framebuffer, label_x + CHANNEL_VALUE_OFFSET, 13,
              "---", COLOR_MUTED, 1);
  }
}

/* 根据接收状态选择颜色并在底部显示 RX OK、CRC ERROR 或 BAD FRAME。 */
static void draw_status(uint16_t *framebuffer, RxVisualState status,
                        const char *detail)
{
  char rate_text[18] = "SAMPLE ---";
  uint16_t color;
  const char *label;

  switch (status)
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
  if (detail != NULL)
  {
    draw_text(framebuffer, 76, 248, detail, COLOR_MUTED, 1);
  }

  if (sample_interval_us > 0U)
  {
    char number[11];

    strcpy(rate_text, "SAMPLE ");
    format_uint(number, sample_interval_us);
    strcat(rate_text, number);
    strcat(rate_text, "US");
  }
  draw_text(framebuffer, 166, 248, rate_text,
            timebase_pending ? COLOR_WARN : COLOR_MUTED, 1);
  if (waveform_paused)
  {
    draw_text(framebuffer, 306, 248, "PAUSED", COLOR_WARN, 1);
  }
  else if (timebase_failed &&
      ((HAL_GetTick() - timebase_failed_at_ms) < 3000U))
  {
    draw_text(framebuffer, 306, 248, "TB FAIL", COLOR_ERROR, 1);
  }
}

/*
 * 通知 LTDC 从哪一块 SDRAM 开始读取像素。
 * 这个操作是双缓冲方式的关键：新画面完全画完后才切换。
 */
static void present_frame(uint16_t *framebuffer)
{
  (void)HAL_LTDC_SetAddress(&hltdc, (uint32_t)framebuffer, 0);
}

/* 上电等待数据时的静态界面。 */
static void render_waiting_screen(void)
{
  uint16_t *framebuffer = framebuffers[active_framebuffer];

  draw_grid(framebuffer);
  draw_channel_reading(framebuffer, 50, CHANNEL_LABEL_CH1,
                       COLOR_CH1, NULL, 0U);
  draw_channel_reading(framebuffer, 168, CHANNEL_LABEL_CH2,
                       COLOR_CH2, NULL, 0U);
  draw_text(framebuffer, 112, 142, "WAITING FOR DATA", COLOR_WARN, 1);
  draw_status(framebuffer, RX_WAITING, NULL);
  present_frame(framebuffer);
}

/*
 * 将最新的 CH1、CH2 数据画到后台缓冲区，然后一次性切换给 LCD 显示。
 * 两块缓冲区交替使用，避免边画边显示造成闪烁。
 */
static void render_received_frame(void)
{
  char sequence_text[12];
  /* 当前前台缓冲区保持显示，新的完整帧画到后台缓冲区。 */
  uint16_t *framebuffer = framebuffers[active_framebuffer ^ 1U];

  draw_grid(framebuffer);
  draw_waveform(framebuffer, samples_ch1, sample_count_ch1, COLOR_CH1);
  draw_waveform(framebuffer, samples_ch2, sample_count_ch2, COLOR_CH2);
  draw_channel_reading(framebuffer, 50, CHANNEL_LABEL_CH1, COLOR_CH1,
                       samples_ch1, sample_count_ch1);
  draw_channel_reading(framebuffer, 168, CHANNEL_LABEL_CH2, COLOR_CH2,
                       samples_ch2, sample_count_ch2);

  sequence_text[0] = 'S';
  sequence_text[1] = 'E';
  sequence_text[2] = 'Q';
  sequence_text[3] = ':';
  format_uint(&sequence_text[4], frame_sequence);
  draw_status(framebuffer, visual_state, sequence_text);

  present_frame(framebuffer);
  /* LTDC 地址切换完成后，交换前后台角色。 */
  active_framebuffer ^= 1U;
}

/* ============================ 6. 数据帧解析 ============================ */
/*
 * 计算 CRC-16/CCITT-FALSE。
 * 发送端和接收端必须使用同样的多项式 0x1021、初值 0xFFFF。
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;
  uint32_t index;
  uint8_t bit;

  for (index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8;
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* 清空当前半帧状态，准备重新寻找下一个 A5 5A 同步头。 */
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

/*
 * PC 上位机 ACK:
 * ACK,<status>,<sequence>,<points>,<interval_us>,<ok>,<crc_errors>,<drops>
 *
 * status 为 OK、CRC 或 FORMAT。最后一个字段是串口环形缓冲区丢字节次数。
 */
static void send_frame_ack(const char *status, uint16_t sequence,
                           uint16_t count, uint16_t interval,
                           BoardLinkSource source)
{
  char response[96];
  uint8_t position = 0U;

  append_text(response, &position, sizeof(response), "ACK,");
  append_text(response, &position, sizeof(response), status);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), sequence);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), count);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), interval);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), valid_frame_count);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response), crc_error_count);
  append_text(response, &position, sizeof(response), ",");
  append_number(response, &position, sizeof(response),
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

/*
 * 向采集端发送时间档位命令。
 * packet[4] 是档位，packet[5..6] 是对应采样间隔（微秒，小端）。
 */
static void send_timebase_command(uint8_t index)
{
  uint8_t packet[9];
  uint16_t interval = timebase_interval_us[index];
  uint16_t crc;

  /*
   * 通知采集端切换采样间隔：
   * index 0 -> 200 us, 1 -> 100 us, 2 -> 20 us。
   */
  packet[0] = CONTROL_SYNC_0;
  packet[1] = CONTROL_SYNC_1;
  packet[2] = CONTROL_VERSION;
  packet[3] = CONTROL_CMD_TIMEBASE;
  packet[4] = index;
  packet[5] = (uint8_t)(interval & 0xFFU);
  packet[6] = (uint8_t)(interval >> 8);
  crc = crc16_ccitt(packet, 7U);
  packet[7] = (uint8_t)(crc & 0xFFU);
  packet[8] = (uint8_t)(crc >> 8);

  {
    /*
     * Use the short line-oriented form on UART7. It is tolerant of a lost
     * byte between retries, unlike a control packet whose CRC covers all 9
     * bytes. The full binary packet is still mirrored to the PC for logging.
     */
    char fallback[6] = {'T', 'B', '=', (char)('0' + index), '\r', '\n'};
    BoardLink_WriteBoard((const uint8_t *)fallback, sizeof(fallback));
  }
  /* ACK/control mirrors let the PC observe board-to-board traffic. */
  BoardLink_WritePc(packet, sizeof(packet));
}

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

/*
 * 所有换档入口都汇聚到这里。命令立即发送，并在 150 ms 后最多重发两次；
 * 收到与请求一致的采样间隔后停止重发，避免把数据帧切换延迟误认为按键失灵。
 */
static void select_timebase(uint8_t index)
{
  if (index >= 3U)
  {
    return;
  }

  timebase_index = index;
  timebase_pending_index = index;
  timebase_pending = true;
  timebase_failed = false;
  timebase_retries_left = TOUCH_COMMAND_RETRIES - 1U;
  timebase_retry_at_ms = HAL_GetTick() + TOUCH_COMMAND_RETRY_MS;
  send_timebase_command(index);
  send_timebase_debug("TB REQ,IDX=", index, timebase_interval_us[index]);
  refresh_timebase_screen();
}

static void poll_timebase_commands(void)
{
  uint32_t now;

  if (!timebase_pending)
  {
    return;
  }

  now = HAL_GetTick();
  if (timebase_retries_left == 0U)
  {
    if ((int32_t)(now - timebase_retry_at_ms) >= 0)
    {
      timebase_pending = false;
      timebase_failed = true;
      timebase_failed_at_ms = now;
      send_timebase_debug("TB FAIL,IDX=", timebase_pending_index,
                          timebase_interval_us[timebase_pending_index]);
      refresh_timebase_screen();
    }
    return;
  }

  if ((int32_t)(now - timebase_retry_at_ms) >= 0)
  {
    send_timebase_command(timebase_pending_index);
    --timebase_retries_left;
    timebase_retry_at_ms = now + TOUCH_COMMAND_RETRY_MS;
  }
}

/* PI11 每次“松开 -> 按下”切换暂停状态，不再修改扫描档位。 */
static void poll_pause_button(void)
{
  bool pressed =
      HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_SET;
  uint32_t now = HAL_GetTick();

  /* 板上 User 按键为高电平有效，35 ms 消抖。 */
  if ((pressed != button_raw_pressed) &&
      ((now - button_last_change_ms) >= BUTTON_DEBOUNCE_MS))
  {
    button_raw_pressed = pressed;
    button_last_change_ms = now;

    if (pressed)
    {
      waveform_paused = !waveform_paused;
      frame_ready = false;
      refresh_timebase_screen();
      BoardLink_WritePc(
          (const uint8_t *)(waveform_paused ? "PAUSE=1\r\n" : "PAUSE=0\r\n"),
          (uint16_t)(sizeof("PAUSE=1\r\n") - 1U));
    }
  }
}

static void poll_timebase_touch(void)
{
  uint16_t x;
  uint16_t y;
  uint32_t now;
  bool pressed;

  if (!touch_ready)
  {
    return;
  }

  pressed = Touchscreen_ReadPoint(&x, &y);
  now = HAL_GetTick();
  if ((pressed == touch_was_down) ||
      ((now - touch_last_change_ms) < TOUCH_DEBOUNCE_MS))
  {
    return;
  }

  touch_was_down = pressed;
  touch_last_change_ms = now;
  if (!pressed ||
      (y < TOUCH_BUTTON_Y0) || (y > TOUCH_BUTTON_Y1))
  {
    return;
  }

  if ((x >= TOUCH_BUTTON_10MS_X0) &&
      (x < (TOUCH_BUTTON_10MS_X0 + TOUCH_BUTTON_WIDTH)))
  {
    select_timebase(0U);
  }
  else if ((x >= TOUCH_BUTTON_5MS_X0) &&
           (x < (TOUCH_BUTTON_5MS_X0 + TOUCH_BUTTON_WIDTH)))
  {
    select_timebase(1U);
  }
  else if ((x >= TOUCH_BUTTON_1MS_X0) &&
           (x < (TOUCH_BUTTON_1MS_X0 + TOUCH_BUTTON_WIDTH)))
  {
    select_timebase(2U);
  }
}

/*
 * 校验并保存一整帧数据。
 * 返回 true 表示 CRC 和数据格式正确，false 表示这一帧被丢弃。
 * 只有函数返回 true，主循环才会根据 frame_ready 刷新屏幕。
 */
static bool validate_and_store_frame(BoardLinkSource source)
{
  uint16_t count = (uint16_t)frame_buffer[6] |
                   ((uint16_t)frame_buffer[7] << 8);
  uint16_t interval = (uint16_t)frame_buffer[8] |
                      ((uint16_t)frame_buffer[9] << 8);
  uint16_t sequence = (uint16_t)frame_buffer[4] |
                      ((uint16_t)frame_buffer[5] << 8);
  uint8_t frame_type = frame_buffer[3];
  uint8_t frame_flags = frame_buffer[10];
  uint16_t payload_bytes;
  uint16_t expected_flags;
  uint16_t received_crc;
  uint16_t calculated_crc;
  uint16_t index;

  if (frame_type == FRAME_TYPE_CH1)
  {
    expected_flags = FRAME_FLAG_CH1;
    payload_bytes = (uint16_t)(2U * count);
  }
  else if (frame_type == FRAME_TYPE_CH2)
  {
    expected_flags = FRAME_FLAG_CH2;
    payload_bytes = (uint16_t)(2U * count);
  }
  else if (frame_type == FRAME_TYPE_CH1_CH2)
  {
    expected_flags = FRAME_FLAG_CH1_CH2;
    payload_bytes = (uint16_t)(4U * count);
  }
  else
  {
    visual_state = RX_FORMAT_ERROR;
    send_frame_ack("FORMAT", sequence, count, interval, source);
    return false;
  }

  if ((frame_flags != expected_flags) || (frame_buffer[11] != 0U))
  {
    visual_state = RX_FORMAT_ERROR;
    send_frame_ack("FORMAT", sequence, count, interval, source);
    return false;
  }
  if ((count == 0U) || (count > APP_FRAME_MAX_SAMPLES) ||
      (interval < 10U) || (interval > 10000U))
  {
    visual_state = RX_FORMAT_ERROR;
    send_frame_ack("FORMAT", sequence, count, interval, source);
    return false;
  }

  received_crc = (uint16_t)frame_buffer[12U + payload_bytes] |
                 ((uint16_t)frame_buffer[13U + payload_bytes] << 8);
  calculated_crc = crc16_ccitt(frame_buffer, 12U + payload_bytes);
  if (received_crc != calculated_crc)
  {
    ++crc_error_count;
    visual_state = RX_CRC_ERROR;
    send_frame_ack("CRC", sequence, count, interval, source);
    return false;
  }

  for (index = 0U; index < count; ++index)
  {
    if ((frame_type == FRAME_TYPE_CH1) ||
        (frame_type == FRAME_TYPE_CH1_CH2))
    {
      uint16_t offset = (frame_type == FRAME_TYPE_CH1_CH2)
                            ? (uint16_t)(4U * index)
                            : (uint16_t)(2U * index);
      uint16_t value = (uint16_t)frame_buffer[12U + offset] |
                       ((uint16_t)frame_buffer[13U + offset] << 8);
      samples_ch1[index] = (value > 3000U) ? 3000U : value;
    }

    if ((frame_type == FRAME_TYPE_CH2) ||
        (frame_type == FRAME_TYPE_CH1_CH2))
    {
      uint16_t offset = (frame_type == FRAME_TYPE_CH1_CH2)
                            ? (uint16_t)((4U * index) + 2U)
                            : (uint16_t)(2U * index);
      uint16_t value = (uint16_t)frame_buffer[12U + offset] |
                       ((uint16_t)frame_buffer[13U + offset] << 8);
      samples_ch2[index] = (value > 3000U) ? 3000U : value;
    }
  }

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
  sample_interval_us = interval;
  /*
   * 等待采集端确认新档位期间，保留用户刚选择的界面状态，避免旧数据帧
   * 把按钮立即“跳回”原档位。收到目标采样间隔后结束等待。
   */
  if (timebase_pending)
  {
    if (interval == timebase_interval_us[timebase_pending_index])
    {
      timebase_index = timebase_pending_index;
      timebase_pending = false;
      timebase_failed = false;
      send_timebase_debug("TB OK,IDX=", timebase_index, interval);
    }
  }
  else if (interval <= 30U)
  {
    timebase_index = 2U;
  }
  else if (interval <= 150U)
  {
    timebase_index = 1U;
  }
  else
  {
    timebase_index = 0U;
  }
  ++valid_frame_count;
  visual_state = RX_FRAME_OK;
  frame_ready = true;
  send_frame_ack("OK", frame_sequence, count, interval, source);
  return true;
}

/*
 * 串口字节状态机。
 * 第 1 阶段等 A5，第 2 阶段等 5A，收到 12 字节帧头后计算总长度，
 * 最后等待 payload 和 2 字节 CRC 全部到齐。
 */
static void parser_push(uint8_t byte, BoardLinkSource source)
{
  /*
   * 文本调试命令与二进制数据帧共用同一串口。TB=0/1/2 会走与触摸按钮
   * 完全相同的换档函数，便于 PC 上位机自动化验证控制包收发。
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
            (pc_command_buffer[3] <= '2'))
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
    if (byte == FRAME_SYNC_0) frame_buffer[frame_length++] = byte;
    return;
  }
  if (frame_length == 1U)
  {
    if (byte == FRAME_SYNC_1)
    {
      frame_buffer[frame_length++] = byte;
    }
    else if (byte != FRAME_SYNC_0)
    {
      frame_length = 0U;
    }
    return;
  }

  frame_buffer[frame_length++] = byte;
  if (frame_length == APP_FRAME_HEADER_SIZE)
  {
    uint16_t count = (uint16_t)frame_buffer[6] |
                     ((uint16_t)frame_buffer[7] << 8);
    uint8_t type = frame_buffer[3];
    bool valid_type = (type == FRAME_TYPE_CH1) ||
                      (type == FRAME_TYPE_CH2) ||
                      (type == FRAME_TYPE_CH1_CH2);
    if ((frame_buffer[2] == FRAME_VERSION) && valid_type &&
        (count != 0U) && (count <= APP_FRAME_MAX_SAMPLES))
    {
      uint16_t channel_words = (type == FRAME_TYPE_CH1_CH2) ? 4U : 2U;
      expected_frame_length = (uint16_t)(
          APP_FRAME_HEADER_SIZE + (channel_words * count) + 2U);
      frame_header_valid = true;
    }
    else
    {
      visual_state = RX_FORMAT_ERROR;
      reset_parser();
      if (byte == FRAME_SYNC_0) frame_buffer[frame_length++] = byte;
    }
  }

  if (frame_header_valid && (frame_length == expected_frame_length))
  {
    (void)validate_and_store_frame(source);
    reset_parser();
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

/*
 * 应用层入口：在 main.c 的外设初始化之后调用一次。
 * 依次完成 MPU、SDRAM 命令序列、LCD 使能、清屏、UART 接收和就绪通知。
 */
void App_ReceiverInit(void)
{
  configure_framebuffer_mpu();
  SDRAM_DeviceInit();

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
  Touchscreen_Init();
  touch_ready = Touchscreen_IsReady();

  memset((void *)framebuffers[0], 0,
         LCD_FRAME_PIXELS * sizeof(uint16_t));
  memset((void *)framebuffers[1], 0,
         LCD_FRAME_PIXELS * sizeof(uint16_t));

  active_framebuffer = 0U;
  timebase_index = 0U;
  timebase_pending = false;
  timebase_pending_index = 0U;
  timebase_retries_left = 0U;
  timebase_failed = false;
  timebase_failed_at_ms = 0U;
  waveform_paused = false;
  pc_command_length = 0U;
  touch_was_down = false;
  touch_last_change_ms = HAL_GetTick();
  button_raw_pressed =
      HAL_GPIO_ReadPin(KEY_USER_GPIO_Port, KEY_USER_Pin) == GPIO_PIN_SET;
  button_last_change_ms = HAL_GetTick();
  present_frame(framebuffers[0]);
  render_waiting_screen();

  BoardLink_Init();
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

/*
 * 应用层主循环：
 * 1. 从环形缓冲区搬运字节给协议状态机；
 * 2. 检查并恢复 UART 错误；
 * 3. 扫描触摸屏或 User 键切换扫描时间，并重发未确认的控制包；
 * 4. 有完整新帧时绘制到 SDRAM 并切换 LTDC 地址。
 */
void App_ReceiverLoop(void)
{
  uint16_t processed = 0U;
  uint8_t value;

  while ((processed < RX_BYTES_PER_LOOP) &&
         BoardLink_ReadBoardByte(&value))
  {
    parser_push(value, BOARD_LINK_SOURCE_BOARD);
    ++processed;
  }

  while ((processed < RX_BYTES_PER_LOOP) &&
         BoardLink_ReadPcByte(&value))
  {
    parser_push(value, BOARD_LINK_SOURCE_PC);
    ++processed;
  }

  BoardLink_Service();
  poll_pause_button();
  poll_timebase_touch();
  poll_timebase_commands();

  if (frame_ready)
  {
    frame_ready = false;
    if (!waveform_paused)
    {
      render_received_frame();
    }
  }

  if (processed == 0U)
  {
    HAL_Delay(1U);
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
