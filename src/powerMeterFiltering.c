#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "powerMeterFiltering.h"

LOG_MODULE_REGISTER(powerMeterFiltering,LOG_LEVEL_INF);


void calculateAvgValue(filterData* filterData)
{
    float sum = 0.0;
    for(int i = 0; i <= sizeof(filterData->valuesBuffer) / sizeof(filterData->valuesBuffer[0]); i++)
    {
        sum += filterData->valuesBuffer[i];
    }

    filterData->avgValue = sum / (sizeof(filterData->valuesBuffer) / sizeof(filterData->valuesBuffer[0]));
}

void calculateMinValue(filterData* filterData)
{
    if (filterData->bufferIndex > 0) 
    {
        // Start with the first valid element as the initial minimum value
        filterData->minValue = filterData->valuesBuffer[0];
   
        // Loop through the entire buffer to test for the minimum value
        for (int i = 0; i < sizeof(filterData->valuesBuffer) / sizeof(filterData->valuesBuffer[0]); i++) 
        {
            if (filterData->valuesBuffer[i] < filterData->minValue) 
            {
                // Update the minValue if the current value is less than the current minValue
                filterData->minValue = filterData->valuesBuffer[i];
            }
        }
    }
}

void calculateMaxValue(filterData* filterData)
{
    for(int i = 0; i < sizeof(filterData->valuesBuffer) / sizeof(filterData->valuesBuffer[0]); i++)
    {
        if(filterData->valuesBuffer[i] > filterData->maxValue)
        {
            // Update the maxValue if the current value is greater than the maxValue
            filterData->maxValue = filterData->valuesBuffer[i];
        }
    }
}

//Add a new value to the buffer and check if the buffer is full
//If it is full reset the index to 0 and add the new value
void addNewValue(float newValue, filterData* filterData)
{
    // Check if the buffer is full
    if(filterData->bufferIndex >= sizeof(filterData->valuesBuffer) / sizeof(filterData->valuesBuffer[0]))
    {
        // Reset the buffer index to 0
        filterData->bufferIndex = 0;
    }
    filterData->valuesBuffer[filterData->bufferIndex] = newValue;

    calculateAvgValue(filterData);
    calculateMaxValue(filterData);
    calculateMinValue(filterData);
    filterData->newestValue = filterData->valuesBuffer[filterData->bufferIndex];
    filterData->OldestValue = filterData->valuesBuffer[0];

    filterData->bufferIndex++;
}

float getNewestValue(filterData* filterData)
{
    return filterData->newestValue;
}

float getOldestValue(filterData* filterData)
{
    return filterData->OldestValue;
}

float getMinValue(filterData* filterData)
{
    return filterData->minValue;
}

float getMaxValue(filterData* filterData)
{
    return filterData->maxValue;
}


float getAvgValue(filterData* filterData)
{
    return filterData->avgValue;
}