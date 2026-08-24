#include "stm32f10x.h"                  // Device header

#include "stdio.h"
#include "string.h"
#include "math.h"

uint16_t coord[10]={0};

uint16_t *String_To_Num(char *a)
{
	int len=strlen(a),i,j,count=0,wei[20],times=0;
	uint8_t  ctoi=0,befctoi=0;
	for(i=0;i<len+1;i++)
	{
		if(a[i]>='0'&&a[i]<='9')
		{
			ctoi=1;
		}
		else
		{
			ctoi=0;
		}
		if(befctoi==0&&ctoi==1)//上升沿
		{
			wei[count]=a[i]-'0';
			befctoi=1;
			count++;
		}
		else if(befctoi==1&&ctoi==1)//高位
		{
			wei[count]=a[i]-'0';
			count++;
		}
		else if(befctoi==1&&ctoi==0)//下降沿
		{
			for(j=0;j<count;j++)
			{
				coord[times]+=wei[j]*pow(10,count-j-1);
			}
			times++;
			befctoi=0;
			count=0;
		}
		
	}
	return coord ;
}
