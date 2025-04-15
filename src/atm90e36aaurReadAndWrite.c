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

#include "atm90e36aaurReadAndWrite.h"

LOG_MODULE_REGISTER(atm90e36aaur,LOG_LEVEL_INF);

#define PM_RESET_N DT_ALIAS(atm90reset)

#define MY_SPI_MASTER DT_NODELABEL(spi_atm90)
#define MY_SPI_MASTER_CS_DT_SPEC SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_spi_a121))

#define SPI_MAX_TRANSFER_SIZE 64

char msgBuffer[128];

static const struct gpio_dt_spec pmResetN = GPIO_DT_SPEC_GET(PM_RESET_N, gpios);

const struct device *spi_dev;

static struct spi_config spi_cfg = 
{
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.frequency = 125000, //This can at the most, in the case of the ATM90E36AAUR, be 150kHz according to datasheet
	.slave = 0,
	.cs = {.gpio = MY_SPI_MASTER_CS_DT_SPEC, .delay = 0},
};

int atm90e36aaurInit()
{
    int err;

	LOG_INF("atm90e36aaurInit");

	spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi_dev))
    {
		LOG_INF("SPI master device not ready!\n");
        return -1;
	}
	struct gpio_dt_spec spim_cs_gpio = MY_SPI_MASTER_CS_DT_SPEC;
	if(!device_is_ready(spim_cs_gpio.port))
    {
		LOG_INF("SPI master chip select device not ready!\n");
        return -1;
	} 

    if (!gpio_is_ready_dt(&pmResetN)) 
	{
		LOG_ERR("Gpio not ready");
		return -1;
	} 

    err = gpio_pin_configure_dt(&pmResetN, GPIO_OUTPUT_ACTIVE);
	if (err < 0) 
	{
		LOG_ERR("Config failed");
		return -1;
	}

    err = gpio_pin_set_dt(&pmResetN,true);
	if (err != 0) 
	{
		LOG_ERR("Pin set failed");
		return -1;
	}

    k_sleep(K_SECONDS(1));

    err = gpio_pin_set_dt(&pmResetN,false);
	if (err != 0) 
	{
		LOG_ERR("Pin set failed");
		return -1;
	}

    k_sleep(K_SECONDS(1));

    err = gpio_pin_set_dt(&pmResetN,true);
	if (err != 0) 
	{
		LOG_ERR("Pin set failed");
		return -1;
	}
    
    return 0;
}


//Takes an address to read from as parameter and then returns the value read
//Do note that the reason the tx buffer uses a array with 4 index declared is due to both a quirk in the SPI Zephyr api
//And the fact that after the 2 byte address does the ATM need 16 additional clock periods to be able to return the data
int atm90e36aaurReadFromAddress(uint16_t registerAddress, char* dataFromAtm)
{
    static uint8_t txBuffer[4];

    const struct spi_buf tx_buf = 
    {
        .buf = txBuffer,
        .len = sizeof(txBuffer),
    };
    const struct spi_buf_set tx = 
    {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = 
    {
        .buf = dataFromAtm,
        .len = sizeof(dataFromAtm),
    };
    const struct spi_buf_set rx = 
    {
        .buffers = &rx_buf,
        .count = 1,
    };

    txBuffer[0]=(uint8_t)((registerAddress & 0xFF00) >> 8);

    //Set the msb to 1 to indicate a read sequence
    txBuffer[0] |= 1 << 7;

    txBuffer[1]=(uint8_t)(registerAddress & 0x00FF);
    txBuffer[2]=0x00;
    txBuffer[3]=0x00;
    
    int err;
    err = spi_transceive(spi_dev,&spi_cfg,&tx, &rx);
    if(err != 0)
    {
        LOG_ERR("Error occured in transceive function!");
        return -1;
    }
        
   k_sleep(K_MSEC(1));

    return 0;
}

int atm90e36aaurWriteToAddress(uint16_t registerAddress, uint16_t data)
{
    int err;
    static uint8_t txBuffer[4];
    uint16_t lastSpiSent;

    const struct spi_buf tx_buf = 
    {
        .buf = txBuffer,
        .len = sizeof(txBuffer),
    };
    const struct spi_buf_set tx = 
    {
        .buffers = &tx_buf,
        .count = 1,
    };

    txBuffer[0] = (uint8_t)((registerAddress & 0x7F00) >> 8);
    //Clear the msb to 0 to indicate a write sequence
    //txBuffer[0] &= ~(1<<7);

    txBuffer[1] = (uint8_t)(registerAddress & 0x00FF);
    txBuffer[2] = (uint8_t)((data & 0xFF00) >> 8);
    txBuffer[3] = (uint8_t)(data & 0x00FF);

    lastSpiSent = txBuffer[2]*256+txBuffer[3];

    err = spi_write(spi_dev,&spi_cfg,&tx);
        if(err != 0)
    {
        LOG_ERR("Error occurred in write function!");
        return -1;
    }
    k_sleep(K_MSEC(1));
    return 0;
}

#define DEBUG_OFFSET_VAL 0xFFFF

void calibrateGain(uint16_t measureAddress, uint16_t measureAddressLsb, uint16_t gainAddress, double reference, uint16_t intFactor, double decimalFactor)
{
    char dataFromAtmBuffer[4];

    double measuredValueDouble = 0;
    double totalMeasuredValue = 0;
    uint16_t measuredHex;
    uint16_t measuredLsbHex;
    uint16_t gainHex;    
    uint16_t returnVal;

    // Take 10 measurements and calculate the average
    for (int i = 0; i < 10; i++)
    {
        atm90e36aaurReadFromAddress(measureAddress, dataFromAtmBuffer);
        measuredHex = (uint16_t)((uint16_t)dataFromAtmBuffer[2] << 8) | (uint16_t)dataFromAtmBuffer[3];
        
        atm90e36aaurReadFromAddress(measureAddressLsb, dataFromAtmBuffer);
        measuredLsbHex = dataFromAtmBuffer[2]; // We're only interested in the higher bits

        measuredValueDouble = ((double)measuredHex * decimalFactor) + (measuredLsbHex * decimalFactor / 256);
        totalMeasuredValue += measuredValueDouble;
        k_sleep(K_MSEC(10));
    }

    double averageMeasuredValue = totalMeasuredValue / 10;

    gainHex = (reference/averageMeasuredValue) * intFactor;

    atm90e36aaurWriteToAddress(gainAddress, gainHex);

    //Debug read from atm to ensure correct value was written
    atm90e36aaurReadFromAddress(gainAddress,dataFromAtmBuffer);

    returnVal = (uint16_t) ((uint16_t) dataFromAtmBuffer[2] << 8) | (uint16_t) dataFromAtmBuffer[3];
    if(returnVal != gainHex)
    {
        LOG_ERR("Gain value read from register does not match the entered value!");
        LOG_ERR("Expected:%x, Got:%x",gainHex,returnVal);
    }
    else
    {
        LOG_INF("Gain value from ATM90E36 value:%x:%x",dataFromAtmBuffer[2],dataFromAtmBuffer[3]);
    }
}

void calibrateOffsetVoltOrCurrent(uint16_t measureAddress, uint16_t offsetAddress)
{
    char dataFromAtmBuffer[4];
    uint16_t measuredHex;
    uint16_t measured2ComplementHex;
    uint16_t returnVal;
    double totalMeasuredValue = 0;

    //Take 10 measurements and calculate the average
    for (int i = 0; i < 10; i++)
    {
        atm90e36aaurReadFromAddress(measureAddress, dataFromAtmBuffer);

        measuredHex = (uint16_t)((uint16_t)dataFromAtmBuffer[2] << 8) | (uint16_t)dataFromAtmBuffer[3];
        totalMeasuredValue += measuredHex;
    }

    uint16_t averageMeasuredHex = (uint16_t)(totalMeasuredValue / 10);

    //Calculate two's complement of the average value
    measured2ComplementHex = ~averageMeasuredHex + 1;

    atm90e36aaurWriteToAddress(offsetAddress, measured2ComplementHex);

    //Debug read from ATM to ensure correct value was written
    atm90e36aaurReadFromAddress(offsetAddress, dataFromAtmBuffer);

    returnVal = (uint16_t)((uint16_t)dataFromAtmBuffer[2] << 8) | (uint16_t)dataFromAtmBuffer[3];
    if (returnVal != measured2ComplementHex)
    {
        LOG_ERR("Offset value read from register does not match the entered value!");
        LOG_ERR("Expected:%x, Got:%x", measured2ComplementHex, returnVal);
    }
    else
    {
        LOG_INF("Offset value from ATM90E36 value:%x:%x", dataFromAtmBuffer[2], dataFromAtmBuffer[3]);
    }
}

void calibrateOffsetPower(uint16_t measureAddress, uint16_t offsetAddress)
{
    char dataFromAtmBuffer[4];

    uint16_t measuredHex;
    uint16_t measured2ComplementHex;

    uint16_t returnVal;

    atm90e36aaurReadFromAddress(measureAddress,dataFromAtmBuffer);

    measuredHex = (uint16_t) ((uint16_t) dataFromAtmBuffer[2] << 8) | (uint16_t) dataFromAtmBuffer[3];

    measured2ComplementHex = ~measuredHex + 1;

    atm90e36aaurWriteToAddress(offsetAddress, measured2ComplementHex);

    //Debug read from atm to ensure correct value was written
    atm90e36aaurReadFromAddress(offsetAddress,dataFromAtmBuffer);

    returnVal = (uint16_t) ((uint16_t) dataFromAtmBuffer[2] << 8) | (uint16_t) dataFromAtmBuffer[3];
    if(returnVal != measured2ComplementHex)
    {
        LOG_ERR("Offset value read from register does not match the entered value!");
        LOG_ERR("Expected:%x, Got:%x",measured2ComplementHex,returnVal);
    }
    else
    {
        LOG_INF("Offset value from ATM90E36 value:%x:%x",dataFromAtmBuffer[2],dataFromAtmBuffer[3]);
    }
}

//Calibrates the ATM, which must be done before normal use
//The useInternalCalibrationValues parameter is used to determine if the internal calibration values should be used or not
//If not used will it require the user to connected a reference voltage, current, power and pf to the ATM90E36A
//If used will the ATM90E36A use the internal calibration values, which are pre calibrated at the factory
int atm90e36aaurCalibration(bool useInternalCalibrationValues)
{
    char dataFromAtmBuffer[4];

    atm90e36aaurWriteToAddress(SoftReset,0x789A);

    atm90e36aaurWriteToAddress(SoftReset, 0x789A);   // Perform soft reset
    atm90e36aaurWriteToAddress(FuncEn0, 0x0000);     // Voltage sag
    atm90e36aaurWriteToAddress(FuncEn1, 0x0000);     // Voltage sag
    atm90e36aaurWriteToAddress(SagTh, 0x0001);       // Voltage sag threshold

    /* SagTh = Vth * 100 * sqrt(2) / (2 * Ugain / 32768) */
    
    //Set metering config values (CONFIG)
    atm90e36aaurWriteToAddress(ConfigStart, 0x5678); // Metering calibration startup 
    atm90e36aaurWriteToAddress(PLconstH, 0x0861);    // PL Constant MSB (default)
    atm90e36aaurWriteToAddress(PLconstL, 0xC468);    // PL Constant LSB (default)
    atm90e36aaurWriteToAddress(MMode0, 0x1087);      // Mode Config (60 Hz, 3P4W)
    atm90e36aaurWriteToAddress(MMode1, 0x0000);      // PGA gain for 4 current channels = 1 and PGA gain for all adc channels = 1x
    atm90e36aaurWriteToAddress(PStartTh, 0x0000);    // Active Startup Power Threshold
    atm90e36aaurWriteToAddress(QStartTh, 0x0000);    // Reactive Startup Power Threshold
    atm90e36aaurWriteToAddress(SStartTh, 0x0000);    // Apparent Startup Power Threshold
    atm90e36aaurWriteToAddress(PPhaseTh, 0x0000);    // Active Phase Threshold
    atm90e36aaurWriteToAddress(QPhaseTh, 0x0000);    // Reactive Phase Threshold
    atm90e36aaurWriteToAddress(SPhaseTh, 0x0000);    // Apparent  Phase Threshold
    atm90e36aaurWriteToAddress(CSZero, 0x4741);      // Checksum 0 MOD(E1D0,)
    
    //Set metering calibration values (CALIBRATION)
    atm90e36aaurWriteToAddress(CalStart, 0x5678);    // Metering calibration startup 
    atm90e36aaurWriteToAddress(GainA, 0x0000);       // Line calibration gain
    atm90e36aaurWriteToAddress(PhiA, 0x0000);        // Line calibration angle
    atm90e36aaurWriteToAddress(GainB, 0x0000);       // Line calibration gain
    atm90e36aaurWriteToAddress(PhiB, 0x0000);        // Line calibration angle
    atm90e36aaurWriteToAddress(GainC, 0x0000);       // Line calibration gain
    atm90e36aaurWriteToAddress(PhiC, 0x0000);        // Line calibration angle
    atm90e36aaurWriteToAddress(PoffsetA, 0x0000);    // A line active power offset
    atm90e36aaurWriteToAddress(QoffsetA, 0x0000);    // A line reactive power offset
    atm90e36aaurWriteToAddress(PoffsetB, 0x0000);    // B line active power offset
    atm90e36aaurWriteToAddress(QoffsetB, 0x0000);    // B line reactive power offset
    atm90e36aaurWriteToAddress(PoffsetC, 0x0000);    // C line active power offset
    atm90e36aaurWriteToAddress(QoffsetC, 0x0000);    // C line reactive power offset
    atm90e36aaurWriteToAddress(CSOne, 0x0000);       // Checksum 1
    
    //Set metering calibration values (HARMONIC)
    atm90e36aaurWriteToAddress(HarmStart, 0x5678);   // Metering calibration startup 
    atm90e36aaurWriteToAddress(POffsetAF, 0x0000);   // A Fund. active power offset
    atm90e36aaurWriteToAddress(POffsetBF, 0x0000);   // B Fund. active power offset
    atm90e36aaurWriteToAddress(POffsetCF, 0x0000);   // C Fund. active power offset
    atm90e36aaurWriteToAddress(PGainAF, 0x0000);     // A Fund. active power gain
    atm90e36aaurWriteToAddress(PGainBF, 0x0000);     // B Fund. active power gain
    atm90e36aaurWriteToAddress(PGainCF, 0x0000);     // C Fund. active power gain
    atm90e36aaurWriteToAddress(CSTwo, 0x0000);       // Checksum 2 
    atm90e36aaurReadFromAddress(SysStatus0,dataFromAtmBuffer);
    LOG_INF("SysStatus0:%x:%x",dataFromAtmBuffer[2],dataFromAtmBuffer[3]);

    //Set measurement calibration values (ADJUST)
    atm90e36aaurWriteToAddress(AdjStart, 0x5678);    // Measurement calibration

    
    if(useInternalCalibrationValues == true)
    {
       //Use internal calibration values
       LOG_INF("Using internal calibration values");
       atm90e36aaurWriteToAddress(UgainA, CALIBRATION_VOLTAGE_GAIN_FACTORY);           // Line calibration gain
       atm90e36aaurWriteToAddress(UoffsetA, CALIBRATION_VOLTAGE_OFFSET_FACTORY);       // Line calibration offset
       atm90e36aaurWriteToAddress(IgainA, CALIBRATION_CURRENT_GAIN_FACTORY);           // Line calibration gain
       atm90e36aaurWriteToAddress(IoffsetA, CALIBRATION_CURRENT_OFFSET_FACTORY);       // Line calibration offset

       atm90e36aaurWriteToAddress(UgainB, CALIBRATION_VOLTAGE_GAIN_FACTORY);           // Line calibration gain
       atm90e36aaurWriteToAddress(UoffsetB, CALIBRATION_VOLTAGE_OFFSET_FACTORY);       // Line calibration offset
       atm90e36aaurWriteToAddress(IgainB, CALIBRATION_CURRENT_GAIN_FACTORY);           // Line calibration gain
       atm90e36aaurWriteToAddress(IoffsetB, CALIBRATION_CURRENT_OFFSET_FACTORY);       // Line calibration offset

       atm90e36aaurWriteToAddress(UgainC, CALIBRATION_VOLTAGE_GAIN_FACTORY);           // Line calibration gain
       atm90e36aaurWriteToAddress(UoffsetC, CALIBRATION_VOLTAGE_OFFSET_FACTORY);       // Line calibration offset
       atm90e36aaurWriteToAddress(IgainC, CALIBRATION_CURRENT_GAIN_FACTORY);           // Line calibration gain
       atm90e36aaurWriteToAddress(IoffsetC, CALIBRATION_CURRENT_OFFSET_FACTORY);       // Line calibration offset

       atm90e36aaurWriteToAddress(IgainN, CALIBRATION_CURRENT_OFFSET_FACTORY_N);       // Line calibration gain
    }
    else
    {
        //Use external calibration values
        LOG_INF("Using external calibration values");
        LOG_INF("Calibrating UgainA");
        calibrateGain(UrmsA, UrmsALSB, UgainA, CALIBRATION_VOLTAGE, 52800, 0.01);

        LOG_INF("Calibrating UoffsetA");
        calibrateOffsetVoltOrCurrent(UrmsA, UoffsetA);

        LOG_INF("Calibrating IgainA");
        calibrateGain(IrmsA, IrmsALSB, IgainA, CALIBRATION_CURRENT, 30000, 0.001);

        LOG_INF("Calibrating IoffsetA");
        calibrateOffsetVoltOrCurrent(IrmsA, IoffsetA);


        LOG_INF("Calibrating UgainB");
        calibrateGain(UrmsB, UrmsBLSB, UgainB, CALIBRATION_VOLTAGE, 52800, 0.01);
        
        LOG_INF("Calibrating UoffsetB");
        calibrateOffsetVoltOrCurrent(UrmsB, UoffsetB);

        LOG_INF("Calibrating IgainB");
        calibrateGain(IrmsB, IrmsBLSB, IgainB, CALIBRATION_CURRENT, 30000, 0.001);

        LOG_INF("Calibrating IoffsetB");
        calibrateOffsetVoltOrCurrent(IrmsB, IoffsetB);


        LOG_INF("Calibrating UgainC");
        calibrateGain(UrmsC, UrmsCLSB, UgainC, CALIBRATION_VOLTAGE, 52800, 0.01);

        LOG_INF("Calibrating UoffsetC");
        calibrateOffsetVoltOrCurrent(UrmsC, UoffsetC);

        LOG_INF("Calibrating IgainC");
        calibrateGain(IrmsC, IrmsCLSB, IgainC, CALIBRATION_CURRENT, 30000, 0.001);

        LOG_INF("Calibrating IoffsetC");
        calibrateOffsetVoltOrCurrent(IrmsC, IoffsetC);


        LOG_INF("Calibrating IgainN");
        calibrateGain(IrmsN1, IrmsN0, IgainN, CALIBRATION_CURRENT, 30000, 0.001);
    }
    

    atm90e36aaurWriteToAddress(CSThree, 0x0000);     // Checksum 3
    atm90e36aaurReadFromAddress(SysStatus0,dataFromAtmBuffer);
    LOG_INF("SysStatus0:%x:%x",dataFromAtmBuffer[2],dataFromAtmBuffer[3]);

    // Done with the configuration
    atm90e36aaurWriteToAddress(ConfigStart, 0x8765);
    atm90e36aaurWriteToAddress(CalStart, 0x8765);    // 0x6886 //0x5678 //8765);
    atm90e36aaurWriteToAddress(HarmStart, 0x8765);   // 0x6886 //0x5678 //8765);    
    atm90e36aaurWriteToAddress(AdjStart, 0x8765);    // 0x6886 //0x5678 //8765);  

    k_sleep(K_MSEC(1000));
    return 0; 
}


/*
From datasheet:
Complement, MSB as the sign bit XX.XXX kW
1LSB corresponds to 1Watt for phase A/B/C, and 4Watt for Total (all-phase-sum)
*/
float convertPmeanToFloat(char* inputData)
{
    int16_t calculatedValueBuffer;
	float calculatedValue;

    calculatedValueBuffer = (inputData[2]*256+inputData[3]);
    //printf("%d:%d:%d:%d\n",inputData[0], inputData[1], inputData[2], inputData[3]);

    if(calculatedValueBuffer & 0x8000)
    {
        calculatedValueBuffer = (calculatedValueBuffer & 0x7FFF) * -1;
    }

    calculatedValue = calculatedValueBuffer;//*0.00032; //1 lsb = 0.00032W
   // calculatedValue = calculatedValue * 1000; //Convert to mW
    

    return calculatedValue;
}

/*
From datasheet:
Signed, MSB as the sign bit X.XXX
LSB is 0.001. Range from -1000 to +1000
*/
float convertPFmeanToFloat(char* inputData)
{
    int16_t calculatedValueBuffer;
	float calculatedValue;

    calculatedValueBuffer = (inputData[2]*256+inputData[3]);

    if(calculatedValueBuffer & 0x8000)
    {
        calculatedValueBuffer = (calculatedValueBuffer & 0x7FFF) * -1;
        
    }

    calculatedValue = (float)calculatedValueBuffer / 1000.0;

    return calculatedValue;
}

/*
From datasheet:
1LSB corresponds to 0.01 V
*/
float convertURmsToFloat(float inputData)
{
    //float calculatedValueBuffer;
	float calculatedValue;

    //calculatedValueBuffer = (inputData[2]*256+inputData[3]);
	calculatedValue = inputData/100;

    return calculatedValue;
}

/*
From datasheet:
unsigned 16-bit integer with unit of 0.001A
1LSB corresponds to 0.001 A 
*/
float convertIRmsToFloat(float inputData)
{
    //float calculatedValueBuffer;
	float calculatedValue;

    //calculatedValueBuffer = (inputData[2]*256+inputData[3]);
	calculatedValue = inputData/1000;

    return calculatedValue;
}

float atm90e36aaurGetURms(int uRmsAddress)
{
    char dataFromAtmBuffer[4];
    uint16_t measuredHex;
    uint16_t measured2ComplementHex;
    uint16_t returnVal;
    double totalMeasuredValue = 0;

    //Take 10 measurements and calculate the average
    for (int i = 0; i < 10; i++)
    {
        atm90e36aaurReadFromAddress(uRmsAddress, dataFromAtmBuffer);

        measuredHex = (uint16_t)((uint16_t)dataFromAtmBuffer[2] << 8) | (uint16_t)dataFromAtmBuffer[3];
        totalMeasuredValue += measuredHex;
    }

    double averageMeasuredValue = totalMeasuredValue / 10;
    
    return(convertURmsToFloat(averageMeasuredValue));
}

//Functions for getting different values from the ATM90E36A
//These functions will read the data from the ATM90E36A, convert it to a float and return it
//The functions will take the address of the register to read from as a parameter
float atm90e36aaurGetIRms(int iRmsAddress)
{
    char dataFromAtmBuffer[4];
    uint16_t measuredHex;
    uint16_t measured2ComplementHex;
    uint16_t returnVal;
    double totalMeasuredValue = 0;

    //Take 10 measurements and calculate the average
    for (int i = 0; i < 10; i++)
    {
        atm90e36aaurReadFromAddress(iRmsAddress, dataFromAtmBuffer);

        measuredHex = (uint16_t)((uint16_t)dataFromAtmBuffer[2] << 8) | (uint16_t)dataFromAtmBuffer[3];
        totalMeasuredValue += measuredHex;
    }

    double averageMeasuredValue = totalMeasuredValue / 10;
    
    return(convertIRmsToFloat(averageMeasuredValue));
}

float atm90e36aaurGetPMean(int pMeanAddress)
{
    char dataFromAtmBuffer[4];
    double totalMeasuredValue = 0;

    for(int i = 0; i < 10; i++)
    {
        atm90e36aaurReadFromAddress(pMeanAddress,dataFromAtmBuffer);
        totalMeasuredValue += convertPmeanToFloat(dataFromAtmBuffer);
    }

    double averageMeasuredValue = totalMeasuredValue / 10;

    return averageMeasuredValue;  
}


float atm90e36aaurGetPFMean(int pfMeanAddress)
{
    char dataFromAtmBuffer[4];
    float totalMeasuredValue = 0;

    for(int i = 0; i < 10; i++)
    {
        atm90e36aaurReadFromAddress(pfMeanAddress,dataFromAtmBuffer);
        totalMeasuredValue += convertPFmeanToFloat(dataFromAtmBuffer);
    }

    double averageMeasuredValue = totalMeasuredValue / 10;

    return averageMeasuredValue;  
}
