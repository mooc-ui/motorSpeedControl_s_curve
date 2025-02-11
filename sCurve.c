#include "sCurve.h"

struct S_CURVE_TYPE sCurve0 = 
{
	.accSet = 1,
	.a_accSet = 0.0015,

};

void SCurveProcess(struct S_CURVE_TYPE *sCurve)
{
	if(sCurve->speedTar != sCurve->lastspeedTar)
	{		
	//这里根据当前位置和目标，判断加速还是减速，以及加速度方向				
		if(sCurve->out > 0.0f)
		{
			if(sCurve->speedTar > sCurve->out)
			{//正向加速
				sCurve->acc_dec = 1;
				sCurve->accDir = 1.0f;
			}
			else
			{//正向减速
				sCurve->acc_dec = 2;
				sCurve->accDir = -1.0f;
			}		
		}
		else if(sCurve->out < 0.0f)
		{
			if(sCurve->speedTar > sCurve->out)
			{//正向减速
				sCurve->acc_dec = 2;
				sCurve->accDir = 1.0f;
			}
			else
			{//反向加速
				sCurve->acc_dec = 1;
				sCurve->accDir = -1.0f;
			}		
		}
		else
		{
			if(sCurve->speedTar > sCurve->out)
			{//正向加速
				sCurve->acc_dec = 1;
				sCurve->accDir = 1.0f;
			}
			else
			{//反向加速
				sCurve->acc_dec = 1;
				sCurve->accDir = -1.0f;
			}					
		}	
	
		if(sCurve->sta == 0)
		{//不需要重规划			
			sCurve->a_accStep = 0.5f * sCurve->accSet * sCurve->accSet / sCurve->a_accSet;
			
			if(fabsf(sCurve->speedTar - sCurve->out) > fabsf(sCurve->a_accStep + sCurve->a_accStep))
			{//目标足够大，加速度能达到最大值，采用七段式
				sCurve->accStep = fabsf(sCurve->speedTar - sCurve->out) - fabsf(sCurve->a_accStep + sCurve->a_accStep);
				
				sCurve->accT = sCurve->accStep / sCurve->accSet;
				
				sCurve->accMaxCnt = (uint16_t)sCurve->accT;
				
				sCurve->accTar = sCurve->accSet * sCurve->accDir;
			}
			else
			{//目标不够大，加速度达不到最大值，采用五段式
				float32_t temp;

				arm_sqrt_f32(fabsf((sCurve->speedTar - sCurve->out) / sCurve->a_accSet), &temp);
										
				sCurve->accMaxCnt = 0;
				
				sCurve->accTar = temp * sCurve->a_accSet * sCurve->accDir;					
			}
			
			sCurve->a_accTemp = sCurve->a_accSet;
			
			sCurve->sta = 1;		
		}
		else if(sCurve->sta != 0)
		{//重规划
			sCurve->a_accStep = 0.5f * sCurve->accSet * sCurve->accSet / sCurve->a_accSet;
			sCurve->a_accStepTemp = 0.5f * (sCurve->accSet * sCurve->accSet - sCurve->acc * sCurve->acc) / sCurve->a_accSet;	
			sCurve->d_accStepTemp = 0.5f * (sCurve->acc * sCurve->acc) / sCurve->a_accSet;
							
			if(fabsf(sCurve->speedTar - sCurve->out) > fabsf(sCurve->a_accStepTemp + sCurve->a_accStep))
			{
				sCurve->accStep = fabsf(sCurve->speedTar - sCurve->out) - fabsf(sCurve->a_accStepTemp + sCurve->a_accStep);
				
				sCurve->accT = sCurve->accStep / sCurve->accSet;
				
				sCurve->accMaxCnt = (uint16_t)sCurve->accT;
				sCurve->accCnt = 0;
				
				sCurve->accTar = sCurve->accSet * sCurve->accDir;
				
				sCurve->a_accTemp = sCurve->a_accSet;		

				sCurve->sta = 1;	
			}
			else if(fabsf(sCurve->speedTar - sCurve->out) > fabsf(sCurve->d_accStepTemp))
			{
				float32_t temp;

				arm_sqrt_f32(fabsf((fabsf(sCurve->speedTar - sCurve->out) + 0.5f * sCurve->acc * sCurve->acc / sCurve->a_accSet) / sCurve->a_accSet), &temp);
										
				sCurve->accMaxCnt = 0;
				sCurve->accCnt = 0;
				
				sCurve->accTar = temp * sCurve->a_accSet * sCurve->accDir;		

				sCurve->a_accTemp = sCurve->a_accSet;			

				sCurve->sta = 1;								
			}
			else
			{
				float32_t temp, accSign;
				
				if(sCurve->acc_dec == 1)
				{
					if(sCurve->acc * sCurve->out > 0.0f)
						accSign = -1.0f;
					else
						accSign = 1.0f;				
				}
				else
				{
					if(sCurve->acc * sCurve->out > 0.0f)
						accSign = 1.0f;
					else
						accSign = -1.0f;							
				}
				
				arm_sqrt_f32(fabsf((fabsf(sCurve->speedTar - sCurve->out) + accSign * 0.5f * sCurve->acc * sCurve->acc / sCurve->a_accSet) / sCurve->a_accSet), &temp);
										
				sCurve->accMaxCnt = 0;
				sCurve->accCnt = 0;
				
				sCurve->accTar = accSign * temp * sCurve->a_accSet * sCurve->accDir;	
		
				sCurve->a_accTemp = sCurve->a_accSet;			

				sCurve->sta = 1;																										
			}			
		}
	
		sCurve->lastspeedTar = sCurve->speedTar;
	}
	
	if(sCurve->sta == 1)
	{		
		if(sCurve->accTar > sCurve->acc)
			sCurve->j = sCurve->a_accTemp;
		else
			sCurve->j = -sCurve->a_accTemp;
		
		if(fabsf(sCurve->accTar - sCurve->acc) < fabsf(sCurve->j))
		{
			sCurve->acc = sCurve->accTar;					
			sCurve->j = 0;
			sCurve->sta = 2;
		}					
	}
	if(sCurve->sta == 2)
	{
		sCurve->j = 0;
		
		sCurve->accCnt ++;
		
		if(sCurve->accCnt >= sCurve->accMaxCnt)
		{
			sCurve->accCnt = 0;
			sCurve->accTar = 0.0f;	
			sCurve->sta = 3;
		}					
	}
	if(sCurve->sta == 3)
	{
		if(sCurve->accTar < sCurve->acc)
			sCurve->j = -sCurve->a_accTemp;
		else
			sCurve->j = sCurve->a_accTemp;
		
		if(fabsf(sCurve->accTar - sCurve->acc) < fabsf(sCurve->j))
		{
			sCurve->acc = sCurve->accTar;					
			sCurve->j = 0;
			sCurve->sta = 0;
			
			sCurve->out = sCurve->speedTar;
		}					
	}

	sCurve->acc += sCurve->j;
				
	sCurve->out += sCurve->acc;					
}









