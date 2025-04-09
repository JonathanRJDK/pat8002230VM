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

	char dataFromAtmBuffer[4];

	printf("Main started\n");
	
	atm90e36aaurInit();
	atm90e36aaurCalibration();

	while(1)
	{
		//ToDo: Split these functions up so that you only fetch one phase at a time
		//Also add a average filter so that the values are more precise. 		
		atm90e36aaurGetURms(&uRmsFromAtm);
		atm90e36aaurGetIRms(&iRmsFromAtm);
		atm90e36aaurGetPMean(&pMeanFromAtm);
		atm90e36aaurGetPFMean(&pFMeanFromAtm);
/*
		printf("U RMS:\t\tPhase A:%.2fV B:%.2fV C:%.2fV\n",
		uRmsFromAtm.uRmsA,uRmsFromAtm.uRmsB,uRmsFromAtm.uRmsC);

		printf("i RMS:\t\tPhase N:%.4fA Phase A:%.4fA B:%.4fA C:%.4fA\n",
		iRmsFromAtm.iRmsN,iRmsFromAtm.iRmsA,iRmsFromAtm.iRmsB,iRmsFromAtm.iRmsC);

		printf("P Mean:\t\tPower total:%.6fW Power phase A:%.6fW Power phase B:%.6fW Power phase C:%.6fW\n",
		pMeanFromAtm.pMeanT,pMeanFromAtm.pMeanA,pMeanFromAtm.pMeanB,pMeanFromAtm.pMeanC);

		printf("PF Mean:\tPower factor total:%.2f Power factor phase A:%.2f Power factor phase B:%.2f Power factor phase C:%.2f\n",
		pFMeanFromAtm.pfMeanT,pFMeanFromAtm.pfMeanA,pFMeanFromAtm.pfMeanB,pFMeanFromAtm.pfMeanC);
		printf("\n");
*/

		printf("Phase A:\nUrms - %.2fV\nIrms - %.3fA\nPmean - %.2fW\nPFmean - %.2f\n", uRmsFromAtm.uRmsA,iRmsFromAtm.iRmsA,pMeanFromAtm.pMeanA,pFMeanFromAtm.pfMeanA);
		printf("\n");
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
