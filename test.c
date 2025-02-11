#include "sCurve.h"

float spdTemp；

//此函数建议在1khz频率的定时器中断里调用
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	//建议目标不要超过3500，否则因为浮点数精度问题累计误差会很大，如果需要更大的建议缩放。
	if(spdTemp > 3500.0f)
		spdTemp = 3500.0f;
	else if(spdTemp < -3500.0f)
		spdTemp = -3500.0f;

	SCurveProcess(&sCurve0);

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
