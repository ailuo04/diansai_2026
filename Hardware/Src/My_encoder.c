#include "My_encoder.h"
#include "tim.h"

volatile My_encoder_debug_t My_encoder_debug_My; /* Keil Watch 可直接观察的单编码器运行数据。 */

static uint16_t My_encoder_last_count_My; /* 上一采样周期的 TIM1 原始计数，仅由 TIM6 中断访问。 */

HAL_StatusTypeDef My_encoder_init_My(void)
{
  /* 启动前统一清零，保证首次差值以零点为基准且调试窗口不会显示历史残留值。 */
  My_encoder_debug_My.running = 0U;
  My_encoder_reset_My();

  /* 编码器模式必须同时使能 CH1/CH2；启动失败时保持 running=0 供调试器识别。 */
  if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
  {
    My_encoder_debug_My.running = 0U;
    return HAL_ERROR;
  }

  My_encoder_debug_My.running = 1U;
  return HAL_OK;
}

void My_encoder_update_My(void)
{
  uint16_t current_count; /* 本周期读取的 TIM1 CNT 快照。 */
  uint16_t wrapped_delta; /* 按 16 位环形计数器计算的无符号差值。 */
  int32_t signed_count;   /* 将硬件 16 位计数解释为以零为中心的有符号位置。 */
  int16_t delta_count;    /* 本周期有符号增量。 */

  /* 未启动时不读取外设，也不增加采样次数，便于从 Watch 窗口直接识别启动失败。 */
  if (My_encoder_debug_My.running == 0U)
  {
    return;
  }

  /*
   * CNT 是 16 位环形寄存器。先保留其位模式，再显式完成符号扩展：0xFFFF
   * 对外表示 -1 而不是 65535，避免反向越过零时在 Watch 窗口显示最大正数。
   */
  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
  signed_count = (current_count <= 32767U)
                   ? (int32_t)current_count
                   : ((int32_t)current_count - 65536L);

  /*
   * 差值同样按 16 位环形空间计算，再显式映射到 -32768~32767。这样正向
   * 65535->0 得到 +1，反向 0->65535 得到 -1，不依赖编译器的有符号转换行为。
   */
  wrapped_delta = (uint16_t)(current_count - My_encoder_last_count_My);
  delta_count = (wrapped_delta <= 32767U)
                  ? (int16_t)wrapped_delta
                  : (int16_t)((int32_t)wrapped_delta - 65536L);
  My_encoder_last_count_My = current_count;

  /* 先保存有符号原始值和周期增量，再累计位置；所有操作均为常数时间，适合中断上下文。 */
  My_encoder_debug_My.raw_count = signed_count;
  My_encoder_debug_My.delta_count = delta_count;
  My_encoder_debug_My.total_count += (int32_t)delta_count;
  My_encoder_debug_My.sample_count++;
}

void My_encoder_reset_My(void)
{
  uint8_t running = My_encoder_debug_My.running; /* 清零前保存启动状态，避免在线复位后停止采样。 */

  /* 本函数不自行开关中断；调用方必须保证不会与 TIM6 更新并发。 */
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  My_encoder_last_count_My = 0U;
  My_encoder_debug_My.raw_count = 0;
  My_encoder_debug_My.delta_count = 0;
  My_encoder_debug_My.total_count = 0;
  My_encoder_debug_My.sample_count = 0U;
  My_encoder_debug_My.running = running;
}
