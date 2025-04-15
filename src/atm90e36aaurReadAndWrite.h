#ifndef ATM90E36A_AU_R_H
#define ATM90E36A_AU_R_H

#include <inttypes.h>
#include "atm90e36aaurRegisterAddress.h"

//Used for calibrating the ATM90E36A if not using the already defined calibration values
//The calibration values are used to calibrate the ATM90E36A to the reference voltage and current
#define CALIBRATION_VOLTAGE 232.0
#define CALIBRATION_CURRENT 0.177

//Factory defined calibration values
#define CALIBRATION_VOLTAGE_GAIN_FACTORY 0xC964
#define CALIBRATION_CURRENT_GAIN_FACTORY 0x0C19
#define CALIBRATION_VOLTAGE_OFFSET_FACTORY 0xA334
#define CALIBRATION_CURRENT_OFFSET_FACTORY 0xF94E
#define CALIBRATION_CURRENT_OFFSET_FACTORY_N 0xB57A

#define USE_INTERNAL_CALIBRATION_VALUES true
#define USE_EXTERNAL_CALIBRATION_VALUES false


#ifdef __cplusplus
extern "C" {
#endif
//Init funciton, sets up the SPI
int atm90e36aaurInit();
//Calibration function, sets up the ATM90E36A for calibration. This is done by writing to the registers in the ATM90E36A.
int atm90e36aaurCalibration(bool useInternalCalibrationValues);

//Mostly helper functions internally in the module, however can be used to "manually" access registers
int atm90e36aaurReadFromAddress(uint16_t registerAddress, char* dataFromAtm);
int atm90e36aaurWriteToAddress(uint16_t registerAddress, uint16_t data);




//Functions that gets different data. They will all fetch, convert and populate a struct with the corresponding data
float atm90e36aaurGetPMean(int pMeanAddress);
float atm90e36aaurGetPFMean(int pfMeanAddress);
float atm90e36aaurGetURms(int uRmsAddress);
float atm90e36aaurGetIRms(int iRmsAddress);

#ifdef __cplusplus
}
#endif

#endif //ATM90E36A_AU_R_H
