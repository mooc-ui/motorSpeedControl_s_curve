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
	//������ݵ�ǰλ�ú�Ŀ�꣬�жϼ��ٻ��Ǽ��٣��Լ����ٶȷ���				
		if(sCurve->out > 0.0f)
		{
			if(sCurve->speedTar > sCurve->out)
			{//�������
				sCurve->acc_dec = 1;
				sCurve->accDir = 1.0f;
			}
			else
			{//�������
				sCurve->acc_dec = 2;
				sCurve->accDir = -1.0f;
			}		
		}
		else if(sCurve->out < 0.0f)
		{
			if(sCurve->speedTar > sCurve->out)
			{//�������
				sCurve->acc_dec = 2;
				sCurve->accDir = 1.0f;
			}
			else
			{//�������
				sCurve->acc_dec = 1;
				sCurve->accDir = -1.0f;
			}		
		}
		else
		{
			if(sCurve->speedTar > sCurve->out)
			{//�������
				sCurve->acc_dec = 1;
				sCurve->accDir = 1.0f;
			}
			else
			{//�������
				sCurve->acc_dec = 1;
				sCurve->accDir = -1.0f;
			}					
		}	
	
		if(sCurve->sta == 0)
		{//����Ҫ�ع滮			
			sCurve->a_accStep = 0.5f * sCurve->accSet * sCurve->accSet / sCurve->a_accSet;
			
			if(fabsf(sCurve->speedTar - sCurve->out) > fabsf(sCurve->a_accStep + sCurve->a_accStep))
			{//Ŀ���㹻�󣬼��ٶ��ܴﵽ���ֵ�������߶�ʽ
				sCurve->accStep = fabsf(sCurve->speedTar - sCurve->out) - fabsf(sCurve->a_accStep + sCurve->a_accStep);
				
				sCurve->accT = sCurve->accStep / sCurve->accSet;
				
				sCurve->accMaxCnt = (uint16_t)sCurve->accT;
				
				sCurve->accTar = sCurve->accSet * sCurve->accDir;
			}
			else
			{//Ŀ�겻���󣬼��ٶȴﲻ�����ֵ���������ʽ
				float32_t temp;

				arm_sqrt_f32(fabsf((sCurve->speedTar - sCurve->out) / sCurve->a_accSet), &temp);
										
				sCurve->accMaxCnt = 0;
				
				sCurve->accTar = temp * sCurve->a_accSet * sCurve->accDir;					
			}
			
			sCurve->a_accTemp = sCurve->a_accSet;
			
			sCurve->sta = 1;		
		}
		else if(sCurve->sta != 0)
		{//�ع滮
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









