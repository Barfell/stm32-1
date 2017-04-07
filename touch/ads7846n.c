/*--------------------------------------------------------------------------
 * ads7846n.c -
 *
 * Author: 185275258 (QQ Group)
 *
 *--------------------------------------------------------------------------*/

#include "ads7846n.h"

//#define USE_HAL_DRV
//#define PRESSURE_DETECTION

#define SPI_TRANSMIT_TIMEOUT (10)
#define TOUCH_ADS7846_SPI SPI3

static uint8_t prvReadAdc(uint8_t channel, uint16_t *p_value);
static uint8_t prvSendReadByte(SPI_TypeDef *pspi, const uint8_t tx, uint8_t *prx);
static osMutexId touch_mutex = 0;
static osMutexDef(touch_mutex);

uint8_t ADS7846N_Init(void)
{
    uint8_t ret = 0;

    __HAL_RCC_SPI3_CLK_ENABLE();

    Dev_Gpio_ReInit(e_Dev_Dout_Touch_CS, GPIO_MODE_OUTPUT_PP);

#if defined (USE_HAL_DRV)

    sSpiHandle.Instance = SPI3;
    sSpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    sSpiHandle.Init.Mode = SPI_MODE_MASTER;
    sSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    sSpiHandle.Init.CLKPolarity = SPI_POLARITY_HIGH;    //SPI_POLARITY_LOW;
    sSpiHandle.Init.CLKPhase = SPI_PHASE_2EDGE;//SPI_PHASE_1EDGE;
    sSpiHandle.Init.NSS = SPI_NSS_SOFT;
    sSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;// 84MHz (APB2) / 4 = 21MHz

    sSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    sSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    sSpiHandle.Init.CRCPolynomial = 7;
    sSpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;

    ret = HAL_SPI_Init(&sSpiHandle);

#else

    SPI_InitTypeDef SPI_InitStructure;

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 45MHz (APB1)

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(TOUCH_ADS7846_SPI, &SPI_InitStructure);

    /*!< Enable the sFLASH_SPI  */
    SPI_Cmd(TOUCH_ADS7846_SPI, ENABLE);

    Dev_Gpio_DOut_TurnOn(e_Dev_Dout_Touch_CS);

#endif

    if (touch_mutex == 0)
        touch_mutex = osMutexCreate(osMutex(touch_mutex));

    return ret;

}

uint8_t ADS7846N_IsDetected(void)
{
    if (Dev_Gpio_DIn_State(e_Dev_DIn_Touch_Detected))
    {
        return 1;
    }

    return 0;
}

static uint8_t SPI_SendReceiveOneByte(SPI_TypeDef* spi, const uint8_t data,
        uint8_t* revData)
{
    uint32_t val = 0;
    uint16_t dataRev;
    val = 0;
    //first tx empty
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (val++ > 0x3000)
        {
            return 0;
        }
    }
    SPI_I2S_SendData(TOUCH_ADS7846_SPI, data);
    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(TOUCH_ADS7846_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (val++ > 0x3000)
        {
            return 0;
        }
    }
    dataRev = SPI_I2S_ReceiveData(TOUCH_ADS7846_SPI);

    if (revData)
        *revData = dataRev;
    return 1;
}

static uint8_t SPI_SendReceiveString(SPI_TypeDef* spi, const uint8_t* ptrData,
        uint8_t* revData, uint16_t size)
{
    uint8_t ret = 0;
    //add semaphore
    osMutexWait(touch_mutex, osWaitForever);
    for (uint16_t i = 0; i < size; i++)
    {
        ret = SPI_SendReceiveOneByte(spi, ptrData[i], &(revData[i]));
    }
    osMutexRelease(touch_mutex);
    return ret;
}

uint8_t ADS7846N_ReadXY(uint16_t *px, uint16_t *py, uint16_t *pressure)
{
    uint16_t x = 0, y = 0;
    uint8_t ret = 0;
#if defined PRESSURE_DETECTION
    uint16_t z1 = 0, z2 = 0;
    static const uint8_t TouchCMDData[9] =
    {   0xD0,0x00,0x90,0x00,0xB0,0x00,0xC0,0x00,0x00,};
#else
    static const uint8_t TouchCMDData_5[5] =
    { 0xD0, 0x00, 0x90, 0x00, 0x00, };
#endif
    uint8_t touchInData[9];

#if defined PRESSURE_DETECTION

#else
    ret = SPI_SendReceiveString(TOUCH_ADS7846_SPI, TouchCMDData_5, touchInData, 5);
#endif    // PRESSURE_DETECTION    if (ret)    {
        x = touchInData[1] << 8;
        x += touchInData[2];

        y = touchInData[3] << 8;
        y += touchInData[4];

#if defined PRESSURE_DETECTION
        z1 = touchInData[5]<<8;
        z1+= touchInData[6];

        z2 = touchInData[7]<<8;
        z2+= touchInData[8];

        uint32_t zTemp = (z2-z1)*x;
        *pressure = ( z2 = (zTemp)/(z1>>3) );
#endif  // PRESSURE_DETECTION    }

    *px = (x >> 3) & 0x0FFF;
    *py = (y >> 3) & 0x0FFF;
    return ret;
}

uint8_t ADS7846N_ReadX(uint16_t *px)
{
    return prvReadAdc(0xD0, px);
}

uint8_t ADS7846N_ReadY(uint16_t *py)
{
    return prvReadAdc(0x90, py);
}

static uint8_t prvSendReadByte(SPI_TypeDef *pspi, const uint8_t tx,
        uint8_t *prx)
{

#if defined (USE_HAL_DRV)
    if (p_rx == NULL)
    {
        return HAL_SPI_Transmit(pspi, &tx, 1, 1*TIMEOUT_PER_BYTE);
    }
    else
    {
        return HAL_SPI_TransmitReceive(pspi, &tx, prx, 1, 1*TIMEOUT_PER_BYTE);
    }

#else

    /*!< Loop while DR register in not empty */
    while (SPI_I2S_GetFlagStatus(pspi, SPI_I2S_FLAG_TXE) == RESET)
        ;

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(pspi, tx);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(pspi, SPI_I2S_FLAG_RXNE) == RESET)
        ;

    /*!< Return the byte read from the SPI bus */
    *prx = SPI_I2S_ReceiveData(pspi);

    return 0;

#endif

}

static uint8_t prvReadAdc(const uint8_t channel, uint16_t *p_value)
{
    uint8_t ret = 0;
    uint8_t rx = 0x00;

#if defined (TOUCH_ADS7846_SPI)
    /** Send channel first */
    prvSendReadByte(TOUCH_ADS7846_SPI, channel, &rx);

    /** Read touch value */
    prvSendReadByte(TOUCH_ADS7846_SPI, 0x00, &rx);
    *p_value = (uint16_t) (rx << 8);
    prvSendReadByte(TOUCH_ADS7846_SPI, 0x00, &rx);
    *p_value |= rx;

    /** Add filter to touch value */
    *p_value = *p_value >> 3;

#else
    uint8_t tx = channel;

    ret = HAL_SPI_TransmitReceive(&sSpiHandle, &tx, &rx, 1, SPI_TRANSMIT_TIMEOUT);
    if (ret == 0)
    {
        tx = 0x00;
        ret = HAL_SPI_TransmitReceive(&sSpiHandle, &tx, &rx, 1, SPI_TRANSMIT_TIMEOUT);
        if (ret == 0)
        {
            *p_value = (uint16_t) (rx << 8);

            ret = HAL_SPI_TransmitReceive(&sSpiHandle, &tx, &rx, 1, SPI_TRANSMIT_TIMEOUT);
            if (ret == 0)
            {
                *p_value |= rx;
                *p_value = *p_value >> 3;
            }

        }
    }
#endif

    return ret;

}

static uint8_t Dev_Touch_ReadTEMP1(uint16_t *px)
{
    return prvReadAdc(0xF4, px);
}

static uint8_t Dev_Touch_ReadTEMP0(uint16_t *px)
{
    return prvReadAdc(0x84, px);
}

uint8_t ADS7846N_ReadTempSensor(float *px)
{
    uint16_t temp1, temp0;  //frank added 2015-9-1
    uint8_t ret = 0;
    float temperature;
    temp0 = 0;
    temp1 = 0;
    temperature = 0.01;
    ret = Dev_Touch_ReadTEMP1(&temp1);
    if (ret == 0)
    {
        ret = Dev_Touch_ReadTEMP0(&temp0);
        temperature = 2.573 * (temp1 - temp0) * (3.3 / 4096 * 1000) - 273;
        *px = temperature;
    }

    return ret;
}
// End of file

