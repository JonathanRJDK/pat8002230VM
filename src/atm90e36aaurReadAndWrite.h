#ifndef ATM90E36A_AU_R_H
#define ATM90E36A_AU_R_H

#include <inttypes.h>
#include "atm90e36aaurRegisterAddress.h"

#define CALIBRATION_VOLTAGE 232.0
#define CALIBRATION_CURRENT 0.177

//Global variables that needs to be accessed outside the modules scope

struct atm90e36aaurPmean
{
    float pMeanT;
    float pMeanA;
    float pMeanB;
    float pMeanC; 
};

struct atm90e36aaurPFmean
{
    float pfMeanT;
    float pfMeanA;
    float pfMeanB;
    float pfMeanC; 
};

struct atm90e36aaurURms
{
    float uRmsA;
    float uRmsB;
    float uRmsC;
};

struct atm90e36aaurIRms
{
    float iRmsN;
    float iRmsA;
    float iRmsB;
    float iRmsC;
};

#ifdef __cplusplus
extern "C" {
#endif
//Init funciton, sets up the SPI
int atm90e36aaurInit();
//Not used atm.
int atm90e36aaurStart();

//Mostly helper functions internally in the module, however can be used to "manually" access registers
int atm90e36aaurReadFromAddress(uint16_t registerAddress, char* dataFromAtm);
int atm90e36aaurWriteToAddress(uint16_t registerAddress, uint16_t data);

int atm90e36aaurCalibration();

//Functions that gets different data. They will all fetch, convert and populate a struct with the corresponding data
int atm90e36aaurGetPMean(struct atm90e36aaurPmean* pMeanFromAtm);
int atm90e36aaurGetPFMean(struct atm90e36aaurPFmean* pFMeanFromAtm);
int atm90e36aaurGetURms(struct atm90e36aaurURms* uRmsFromAtm);
int atm90e36aaurGetIRms(struct atm90e36aaurIRms* iRmsFromAtm);

#ifdef __cplusplus
}
#endif

#endif //ATM90E36A_AU_R_H
