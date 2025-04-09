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

#define PM_RESET_N DT_ALIAS(led1)

#define MY_SPI_MASTER DT_NODELABEL(spi_a121)
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
	.cs = {.gpio = MY_SPI_MASTER_CS_DT_SPEC, .delay = 0}, //Datasheet recommends 2 ns, but 1 us is the lowest available value
};

//Internal module functions


//Global functions

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

int atm90e36aaurStart()
{
    return 0;
}

//Takes an address to read from as parameter and then returns the value read
//Do note that the reason the tx buffer uses a array with 4 index declared is due to both a quirk in the SPI Zephyr api
//And the fact that after the 2 byte address does the ATM need 16 additional clock periods to be able to return the data
int atm90e36aaurReadFromAddress(uint16_t registerAddress, char* dataFromAtm)
{
    //static uint8_t rxBuffer[2];
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
    //printf("txBuffer0:%d\n",txBuffer[0]);
    //Set the msb to 1 to indicate a read sequence
    txBuffer[0] |= 1 << 7;

    txBuffer[1]=(uint8_t)(registerAddress & 0x00FF);
    txBuffer[2]=0x00;
    txBuffer[3]=0x00;
    

    //printf("txBuffer1:%d\n",txBuffer[1]);

    int err;
    err = spi_transceive(spi_dev,&spi_cfg,&tx, &rx);
    if(err != 0)
    {
        LOG_ERR("Error occured in transceive function!");
        return -1;
    }
    /*
    printf("Sent:");
    printf("%x:%x:%x:%x",txBuffer[0],txBuffer[1],txBuffer[2],txBuffer[3]);
    //printf("%x",txBuffer[0]*256+txBuffer[1]);
    printf("\n");

    printf("Received:");
    printf("%x:%x:%x:%x",dataFromAtm[0],dataFromAtm[1],dataFromAtm[2],dataFromAtm[3]);
    //printf("%x",dataFromAtm[0]*256+dataFromAtm[1]);
    printf("\n");
    printf("\n");
    */
   k_sleep(K_MSEC(10));

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

    txBuffer[0] = (uint8_t)((registerAddress & 0xFF00) >> 8);
    //Clear the msb to 0 to indicate a write sequence
    txBuffer[0] &= ~(1<<7);

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
    k_sleep(K_MSEC(10));
    return 0;
}

//Calibrates the ATM, which must be done before normal use
int atm90e36aaurCalibration()
{
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
    atm90e36aaurWriteToAddress(MMode0, 0x0007);      // Mode Config (50 Hz, 3P4W)
    atm90e36aaurWriteToAddress(MMode1, 0x1500);      // 0x5555 (x2) // 0x0000 (1x)
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

    //Set measurement calibration values (ADJUST)
    atm90e36aaurWriteToAddress(AdjStart, 0x5678);    // Measurement calibration
    atm90e36aaurWriteToAddress(UgainA, 0x0002);      // A SVoltage rms gain
    atm90e36aaurWriteToAddress(IgainA, 0xFD7F);      // A line current gain
    atm90e36aaurWriteToAddress(UoffsetA, 0x0000);    // A Voltage offset
    atm90e36aaurWriteToAddress(IoffsetA, 0x0000);    // A line current offset
    atm90e36aaurWriteToAddress(UgainB, 0x0002);      // B Voltage rms gain
    atm90e36aaurWriteToAddress(IgainB, 0xFD7F);      // B line current gain
    atm90e36aaurWriteToAddress(UoffsetB, 0x0000);    // B Voltage offset
    atm90e36aaurWriteToAddress(IoffsetB, 0x0000);    // B line current offset
    atm90e36aaurWriteToAddress(UgainC, 0x0002);      // C Voltage rms gain
    atm90e36aaurWriteToAddress(IgainC, 0xFD7F);      // C line current gain
    atm90e36aaurWriteToAddress(UoffsetC, 0x0000);    // C Voltage offset
    atm90e36aaurWriteToAddress(IoffsetC, 0x0000);    // C line current offset
    atm90e36aaurWriteToAddress(IgainN, 0xFD7F);      // C line current gain
    atm90e36aaurWriteToAddress(CSThree, 0x02F6);     // Checksum 3

    // Done with the configuration
    atm90e36aaurWriteToAddress(ConfigStart, 0x5678);
    atm90e36aaurWriteToAddress(CalStart, 0x5678);    // 0x6886 //0x5678 //8765);
    atm90e36aaurWriteToAddress(HarmStart, 0x5678);   // 0x6886 //0x5678 //8765);    
    atm90e36aaurWriteToAddress(AdjStart, 0x5678);    // 0x6886 //0x5678 //8765);  

    atm90e36aaurWriteToAddress(SoftReset, 0x789A);   // Perform soft reset 

    return 0; 
}


/*
From datasheet:
Complement, MSB as the sign bit XX.XXX kW
1LSB corresponds to 1Watt for phase A/B/C, and 4Watt for Total (all-phase-sum)
*/
float convertPmeanToFloat(char* inputData)
{
    uint16_t calculatedValueBuffer;
	float calculatedValue;

    calculatedValueBuffer = (inputData[2]*256+inputData[3]) << 1;

    if(calculatedValueBuffer & 0x8000)
    {
        calculatedValueBuffer = (calculatedValueBuffer & 0x7FFF) * -1;
    }

    calculatedValueBuffer = calculatedValueBuffer/1000;

    return 0;
}

/*
From datasheet:
Signed, MSB as the sign bit X.XXX
LSB is 0.001. Range from -1000 to +1000
*/
float convertPFmeanToFloat(char* inputData)
{
    uint16_t calculatedValueBuffer;
	float calculatedValue;

    calculatedValueBuffer = (inputData[2]*256+inputData[3]) << 1;

    if(calculatedValueBuffer & 0x8000)
    {
        calculatedValueBuffer = (calculatedValueBuffer & 0x7FFF) * -1;
    }

    calculatedValueBuffer = calculatedValueBuffer/1000;

    return 0;
}

/*
From datasheet:
1LSB corresponds to 0.01 V
*/
float convertURmsToFloat(char* inputData)
{
    uint16_t calculatedValueBuffer;
	float calculatedValue;

    calculatedValueBuffer = (inputData[2]*256+inputData[3]) << 1;
    //printf("TEST:%d : %d\n",inputData[2],inputData[3]);
	calculatedValue = calculatedValueBuffer/100;
    //printf("TEST FLOAT: %.2f\n",calculatedValue);
    return calculatedValue;
}

/*
From datasheet:
unsigned 16-bit integer with unit of 0.001A
1LSB corresponds to 0.001 A 
*/
float convertIRmsToFloat(char* inputData)
{
    uint16_t calculatedValueBuffer;
	float calculatedValue;

    calculatedValueBuffer = (inputData[2]*256+inputData[3]) << 1;
	calculatedValue = calculatedValueBuffer/1000;

    return calculatedValue;
}

int atm90e36aaurGetPMean(struct atm90e36aaurPmean* pMeanFromAtm)
{
    char dataFromAtmBuffer[4]; //There should be no need to clear this between uses, since the chip always should give back 16 bits over SPI. Meaning it will be overwritten

    //First get the data from the chip, then convert it to a float value, then populate the atm90e36aaurPmean struct with the data
    //The following code could be made using loops, however this, to approach makes it more readable...
    //The above comments are also applicable to the functions to read the other values

    //Mean total power
    atm90e36aaurReadFromAddress(PmeanTF,dataFromAtmBuffer);
    pMeanFromAtm->pMeanT = convertPmeanToFloat(dataFromAtmBuffer);

    //Mean power through phase A
    atm90e36aaurReadFromAddress(PmeanAF,dataFromAtmBuffer);
    pMeanFromAtm->pMeanA = convertPmeanToFloat(dataFromAtmBuffer);

    //Mean power through phase B
    atm90e36aaurReadFromAddress(PmeanBF,dataFromAtmBuffer);
    pMeanFromAtm->pMeanB = convertPmeanToFloat(dataFromAtmBuffer);

    //Mean power through phase C
    atm90e36aaurReadFromAddress(PmeanCF,dataFromAtmBuffer);
    pMeanFromAtm->pMeanC = convertPmeanToFloat(dataFromAtmBuffer);

    return 0;
}

int atm90e36aaurGetPFMean(struct atm90e36aaurPFmean* pFMeanFromAtm)
{
    char dataFromAtmBuffer[4];

    //Mean total power factor 
    atm90e36aaurReadFromAddress(PFmeanT,dataFromAtmBuffer);
    pFMeanFromAtm->pfMeanT = convertPFmeanToFloat(dataFromAtmBuffer);

    //Mean power factor on phase A
    atm90e36aaurReadFromAddress(PFmeanA,dataFromAtmBuffer);
    pFMeanFromAtm->pfMeanA = convertPFmeanToFloat(dataFromAtmBuffer);

    //Mean power factor on phase B
    atm90e36aaurReadFromAddress(PFmeanB,dataFromAtmBuffer);
    pFMeanFromAtm->pfMeanB = convertPFmeanToFloat(dataFromAtmBuffer);

    //Mean power factor on phase C
    atm90e36aaurReadFromAddress(PFmeanC,dataFromAtmBuffer);
    pFMeanFromAtm->pfMeanC = convertPFmeanToFloat(dataFromAtmBuffer);

    return 0;
}

int atm90e36aaurGetURms(struct atm90e36aaurURms* uRmsFromAtm)
{
    char dataFromAtmBuffer[4];

    //Mean total power factor 
    atm90e36aaurReadFromAddress(UrmsA,dataFromAtmBuffer);
    //printf("TEST:%d : %d\n",dataFromAtmBuffer[2],dataFromAtmBuffer[3]);
    uRmsFromAtm->uRmsA = convertURmsToFloat(dataFromAtmBuffer);
    //printf("FLOAT IN STRUCT:%.2f\n",uRmsFromAtm->uRmsA);

    atm90e36aaurReadFromAddress(UrmsB,dataFromAtmBuffer);
    uRmsFromAtm->uRmsB = convertURmsToFloat(dataFromAtmBuffer);

    atm90e36aaurReadFromAddress(UrmsC,dataFromAtmBuffer);
    uRmsFromAtm->uRmsC = convertURmsToFloat(dataFromAtmBuffer);

    return 0;
}

int atm90e36aaurGetIRms(struct atm90e36aaurIRms* iRmsFromAtm)
{
    char dataFromAtmBuffer[4];

    //Mean total power factor 
    atm90e36aaurReadFromAddress(IrmsN0,dataFromAtmBuffer);
    iRmsFromAtm->iRmsN = convertIRmsToFloat(dataFromAtmBuffer);

    atm90e36aaurReadFromAddress(IrmsA,dataFromAtmBuffer);
    iRmsFromAtm->iRmsA = convertIRmsToFloat(dataFromAtmBuffer);

    atm90e36aaurReadFromAddress(IrmsB,dataFromAtmBuffer);
    iRmsFromAtm->iRmsB = convertIRmsToFloat(dataFromAtmBuffer);

    atm90e36aaurReadFromAddress(IrmsC,dataFromAtmBuffer);
    iRmsFromAtm->iRmsC = convertIRmsToFloat(dataFromAtmBuffer);

    return 0;
}
