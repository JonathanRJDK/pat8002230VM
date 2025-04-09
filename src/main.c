#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include "atm90e36aaurReadAndWrite.h"

char receiveBuffer[4];

struct atm90e36aaurURms uRmsFromAtm;
struct atm90e36aaurIRms iRmsFromAtm; 
struct atm90e36aaurPFmean pFMeanFromAtm;
struct atm90e36aaurPmean pMeanFromAtm;

int main(void)
{
	uint16_t calculatedValueBuffer;
	float calculatedValue;

	printf("Main started\n");
	
	atm90e36aaurInit();
	atm90e36aaurCalibration();

	while(1)
	{
		atm90e36aaurGetURms(&uRmsFromAtm);
		atm90e36aaurGetIRms(&iRmsFromAtm);
		atm90e36aaurGetPMean(&pMeanFromAtm);
		atm90e36aaurGetPFMean(&pFMeanFromAtm);
		
		printf("U RMS:\t\tPhase A:%.2f B:%.2f C:%.2f\n",
		uRmsFromAtm.uRmsA,uRmsFromAtm.uRmsB,uRmsFromAtm.uRmsC);

		printf("i RMS:\t\tPhase N:%.2f Phase A:%.2f B:%.2f C:%.2f\n",
		iRmsFromAtm.iRmsN,iRmsFromAtm.iRmsA,iRmsFromAtm.iRmsB,iRmsFromAtm.iRmsC);

		printf("P Mean:\t\tPower total:%.2f Power phase A:%.2f Power phase B:%.2f Power phase C:%.2f\n",
		pMeanFromAtm.pMeanT,pMeanFromAtm.pMeanA,pMeanFromAtm.pMeanB,pMeanFromAtm.pMeanC);

		printf("PF Mean:\tPower factor total:%.2f Power factor phase A:%.2f Power factor phase B:%.2f Power factor phase C:%.2f\n",
		pFMeanFromAtm.pfMeanT,pFMeanFromAtm.pfMeanA,pFMeanFromAtm.pfMeanB,pFMeanFromAtm.pfMeanC);

		k_sleep(K_MSEC(1000));
	}
	return 0;
}
