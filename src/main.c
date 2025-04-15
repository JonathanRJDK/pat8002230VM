#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#include "atm90e36aaurReadAndWrite.h"
#include "powerMeterFiltering.h"

filterData uRmsA;
filterData iRmsA;
filterData pMeanA;
filterData pfMeanA;

int main(void)
{
	printf("Main started\n");
	
	atm90e36aaurInit();
	atm90e36aaurCalibration(USE_INTERNAL_CALIBRATION_VALUES);

	while(1)
	{
		//URms
		addNewValue(atm90e36aaurGetURms(UrmsA), &uRmsA);

		//IRms
		addNewValue(atm90e36aaurGetIRms(IrmsA), &iRmsA);

		//PMean
		addNewValue(atm90e36aaurGetPMean(PmeanA), &pMeanA);

		//PFMean
		addNewValue(atm90e36aaurGetPFMean(PFmeanA), &pfMeanA);
		
		printf("Urms : %.2fV - Max %.2f - Min %.2f\nIrms : %.3fA - Max %.2f - Min %.2f\nPmean : %.2fW - Max %.2f - Min %.2f\nPFmean : %.2f - Max %.2f - Min %.2f\n", 
			getAvgValue(&uRmsA), getMaxValue(&uRmsA), getMinValue(&uRmsA),
			getAvgValue(&iRmsA), getMaxValue(&iRmsA), getMinValue(&iRmsA), 
			getAvgValue(&pMeanA), getMaxValue(&pMeanA), getMinValue(&pMeanA),
			getAvgValue(&pfMeanA), getMaxValue(&pfMeanA), getMinValue(&pfMeanA)
		);
		
		printf("\n");
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
