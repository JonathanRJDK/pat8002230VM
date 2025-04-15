#ifndef POWER_METER_FILTERING_H
#define POWER_METER_FILTERING_H

//Global macros used by the .c module which needs to easily be modified by the user

//Include libraries needed for the header to compile, often simple libraries like inttypes.h
#include <inttypes.h>

//Global variables that needs to be accessed outside the modules scope
//Filter structure used to filter the power meter values
//This needs to be declared for each value that should be filtered
typedef struct 
{
    float newValue; //New value from the power meter

    float newestValue; //Current value from the power meter
    float OldestValue; //Previous value from the power meter

    float minValue; //Min value of the variable
    float maxValue; //Max value of the variable
    float avgValue; //Average value of the variable

    float valuesBuffer[16]; //Buffer for the values to be filtered
    uint8_t bufferIndex; //Index for the buffer

} filterData;



#ifdef __cplusplus
extern "C" {
#endif
//Functions that should be accessible from the outside 

void addNewValue(float newValue, filterData* filterData);

float getNewestValue(filterData* filterData);

float getOldestValue(filterData* filterData);

float getMinValue(filterData* filterData);

float getMaxValue(filterData* filterData);

float getAvgValue(filterData* filterData);

#ifdef __cplusplus
}
#endif

#endif //POWER_METER_FILTERING_H
