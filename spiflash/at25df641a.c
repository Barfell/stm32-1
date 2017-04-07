/*--------------------------------------------------------------------------
 * spiflash_at25df641a.c -
 *
 * Author: 185275258 (QQ Group)
 *
 *--------------------------------------------------------------------------*/

#include "spiflash.h"

/*#define USE_HAL_DRV */
#if defined (USE_HAL_DRV)
    #define TIMEOUT_PER_BYTE (10)   // 10ms    static SPI_HandleTypeDef sSpiHandle;
#else
    #define sFLASH_SPI (SPI6)
#endif

/** this mutex will give file system re-entrancy from bottom level
 *
 *  mutex init : SpiFlash_Init()
 *  mutex protection:
 *    1. SpiFlash_ReadWrite() for Sector/Page Read/Write operation
 *    2. flash_block_erase() for Sector/Block erase operation
 *    3. SpiFlash_ReadID() for ReadID operation
 *
 * */
static osMutexId spi_mutex;
static osMutexDef(spi_mutex);

static void sFLASH_LowLevel_Init(void);
static void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr,
        uint16_t NumByteToWrite, CS_PIN cs_pin);
static uint8_t sFLASH_UnProtectGlobal(CS_PIN cs_pin);
static uint8_t sFLASH_SendByte(const uint8_t tx, uint8_t *p_rx);
static void sFLASH_WriteEnable(CS_PIN cs_pin);
static void sFLASH_WaitForWriteEnd(CS_PIN cs_pin);

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @param  None
 * @retval None
 */
uint8_t SpiFlash_Init(void)
{
    uint8_t ret = 0;

    spi_mutex = osMutexCreate(osMutex(spi_mutex));

    /** spi controller init */
    sFLASH_LowLevel_Init();

    /** globally un-protect the spi flash */
    sFLASH_UnProtectGlobal(CS1);
    sFLASH_UnProtectGlobal(CS2);

    /** de-select the FLASH: Chip Select high */
    sFLASH_CS_HIGH(CS1);
    sFLASH_CS_HIGH(CS2);

    return ret;
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 * @retval None
 */
static void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr,
        uint16_t NumByteToWrite, CS_PIN cs_pin)
{
#if defined (USE_HAL_DRV)
    uint8_t cmd[4] =
    {
        sFLASH_CMD_WRITE,
        (WriteAddr & 0xFF0000) >> 16,
        (WriteAddr & 0xFF00) >> 8,
        WriteAddr & 0xFF
    };
#endif
    /*!< Enable the write access to the FLASH */
    sFLASH_WriteEnable(cs_pin);

    // un-protect the current sector first
    //sFLASH_UnProtectSector(WriteAddr);

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW(cs_pin);
    /*!< Send "Write to Memory " instruction */
#if defined (USE_HAL_DRV)
    HAL_SPI_Transmit(&sSpiHandle, cmd, 4, 4*TIMEOUT_PER_BYTE);
#else
    sFLASH_SendByte(sFLASH_CMD_WRITE, NULL);
    /*!< Send WriteAddr high nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16, NULL);
    /*!< Send WriteAddr medium nibble address byte to write to */
    sFLASH_SendByte((WriteAddr & 0xFF00) >> 8, NULL);
    /*!< Send WriteAddr low nibble address byte to write to */
    sFLASH_SendByte(WriteAddr & 0xFF, NULL);
#endif
    /*!< while there is data to be written on the FLASH */
#if defined (USE_HAL_DRV)
    HAL_SPI_Transmit(&sSpiHandle, pBuffer, NumByteToWrite, NumByteToWrite * TIMEOUT_PER_BYTE);
#else
    while (NumByteToWrite--)
    {
        sFLASH_SendByte(*pBuffer, NULL);    // Send the current byte
        pBuffer++;    // Point on the next byte to be written
    }
#endif

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH(cs_pin);

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd(cs_pin);

}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 * @retval None
 */
uint8_t SpiFlash_ReadWrite(const uint8_t *txBuf, uint8_t *rxBuf,
        		   uint32_t addr, uint16_t rwBytes, CS_PIN cs)
{
    uint8_t ret = 0;
    uint8_t *pBuffer = NULL;
#if defined (USE_HAL_DRV)
    uint8_t cmd[4] =
    {
        sFLASH_CMD_READ,
        (ReadAddr & 0xFF0000) >> 16,
        (ReadAddr& 0xFF00) >> 8,
        (ReadAddr & 0xFF)
    };
#endif

    if ((txBuf == NULL && rxBuf == NULL) || rwBytes == 0)
        return ret;

    osMutexWait(spi_mutex, osWaitForever);

    if (txBuf == NULL)  // this is a read command
    {
        pBuffer = rxBuf;

        /*!< Select the FLASH: Chip Select low */
        CS_ON(cs);
#if defined (USE_HAL_DRV)
        HAL_SPI_Transmit(&sSpiHandle, cmd, 4, 4*TIMEOUT_PER_BYTE);
        memset(pBuffer, sFLASH_DUMMY_BYTE, NumByteToRead);
        ret |= HAL_SPI_Receive(&sSpiHandle, pBuffer, NumByteToRead, NumByteToRead * TIMEOUT_PER_BYTE);
#else
        /*!< Send "Read from Memory " instruction */
        ret = sFLASH_SendByte(sFLASH_CMD_READ, NULL);
        /*!< Send ReadAddr high nibble address byte to read from */
        ret |= sFLASH_SendByte((addr & 0xFF0000) >> 16, NULL);
        /*!< Send ReadAddr medium nibble address byte to read from */
        ret |= sFLASH_SendByte((addr & 0xFF00) >> 8, NULL);
        /*!< Send ReadAddr low nibble address byte to read from */
        ret |= sFLASH_SendByte(addr & 0xFF, NULL);

        while (rwBytes--) /*!< while there is data to be read */
        {
            ret = sFLASH_SendByte(sFLASH_DUMMY_BYTE, pBuffer); // Read a byte from the FLASH
            pBuffer++; // Point to the next location where the byte read will be saved
        }
#endif
        CS_OFF(cs);
    }
    else    // this is a write command
    {
        pBuffer = (uint8_t *) txBuf;

        sFLASH_WriteEnable(cs);

        /*!< Select the FLASH: Chip Select low */
        CS_ON(cs);
        /*!< Send "Write to Memory " instruction */
#if defined (USE_HAL_DRV)
        HAL_SPI_Transmit(&sSpiHandle, cmd, 4, 4*TIMEOUT_PER_BYTE);
        HAL_SPI_Transmit(&sSpiHandle, pBuffer, NumByteToWrite, NumByteToWrite * TIMEOUT_PER_BYTE);
#else
        sFLASH_SendByte(sFLASH_CMD_WRITE, NULL);
        /*!< Send WriteAddr high nibble address byte to write to */
        sFLASH_SendByte((addr & 0xFF0000) >> 16, NULL);
        /*!< Send WriteAddr medium nibble address byte to write to */
        sFLASH_SendByte((addr & 0xFF00) >> 8, NULL);
        /*!< Send WriteAddr low nibble address byte to write to */
        sFLASH_SendByte(addr & 0xFF, NULL);

        while (rwBytes--) /*!< while there is data to be written on the FLASH */
        {
            sFLASH_SendByte(*pBuffer, NULL);    // Send the current byte
            pBuffer++;    // Point on the next byte to be written
        }
#endif

        /*!< Deselect the FLASH: Chip Select high */
        CS_OFF(cs);

        /*!< Wait the end of Flash writing */
        sFLASH_WaitForWriteEnd(cs);

    }

#if 0
    // todo : need to confirm below comment:
    /*NOTE: different read command need different dummy list,MUST refer to datasheet.*/
    /*Dummy value*/
#if (FLASH_CMD_READ==FLASH_CMD_READ1)
    cmd[cmdLen++]=0;
#elif (FLASH_CMD_READ==FLASH_CMD_READ2)
    cmd[cmdLen++]=0;
    cmd[cmdLen++]=0;
#endif

#endif

    osMutexRelease(spi_mutex);

    return ret;
}

/**
 * @brief  Reads FLASH identification.
 * @param  None
 * @retval FLASH identification
 */
uint8_t SpiFlash_ReadID(CS_PIN cs_pin, uint32_t *p_id)
{
    uint32_t Temp0 = 0, Temp1 = 0, Temp2 = 0;
    uint8_t ret = 0;

    osMutexWait(spi_mutex, osWaitForever);
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW(cs_pin);

    /*!< Send "RDID " instruction */
    ret = sFLASH_SendByte(0x9F, NULL);

    /*!< Read a byte from the FLASH */
    ret |= sFLASH_SendByte(sFLASH_DUMMY_BYTE, &Temp0);

    /*!< Read a byte from the FLASH */
    ret |= sFLASH_SendByte(sFLASH_DUMMY_BYTE, &Temp1);

    /*!< Read a byte from the FLASH */
    ret |= sFLASH_SendByte(sFLASH_DUMMY_BYTE, &Temp2);

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH(cs_pin);
    osMutexRelease(spi_mutex);

    *p_id = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return ret;
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @retval The value of the received byte.
 */
static uint8_t sFLASH_SendByte(const uint8_t tx, uint8_t *p_rx)
{
#if defined (USE_HAL_DRV)
    if (p_rx == NULL)
    {
        return HAL_SPI_Transmit(&sSpiHandle, &tx, 1, 1*TIMEOUT_PER_BYTE);
    }
    else
    {
        return HAL_SPI_TransmitReceive(&sSpiHandle, &tx, p_rx, 1, 1*TIMEOUT_PER_BYTE);
    }

#else
    int val = 0;
    /*!< Loop while DR register in not empty */
    while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (val++ > 0x3000)
        {
            break;
        }
    }
    val = 0;
    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(sFLASH_SPI, tx);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (val++ > 0x3000)
        {
            break;
        }
    }

    /*!< Return the byte read from the SPI bus */
    *p_rx = SPI_I2S_ReceiveData(sFLASH_SPI);

    return 0;

#endif

}

/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @retval None
 */
static void sFLASH_WriteEnable(CS_PIN cs_pin)
{
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW(cs_pin);

    /*!< Send "Write Enable" instruction */
    sFLASH_SendByte(sFLASH_CMD_WREN, NULL);

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH(cs_pin);
}

/**
 * @brief  Global protect or un-protect.
 * @param  need_protect : SET means protect, RESET means un-protect.
 * @retval None
 */
uint8_t sFLASH_UnProtectGlobal(CS_PIN cs_pin)
{
    uint8_t ret = 0;
#if defined (USE_HAL_DRV)
    uint8_t cmd[2] =
    {   sFLASH_CMD_WRSR, 0x00};
#endif
    sFLASH_WriteEnable(cs_pin);

    //sFLASH_WaitForWriteEnd(cs_pin);

    sFLASH_CS_LOW(cs_pin);

#if defined (USE_HAL_DRV)
    ret = HAL_SPI_Transmit(&sSpiHandle, cmd, 2, 2*TIMEOUT_PER_BYTE);
#else
    ret = sFLASH_SendByte(sFLASH_CMD_WRSR, NULL);
    ret |= sFLASH_SendByte(0x00, NULL);
#endif

    sFLASH_CS_HIGH(cs_pin);

    //sFLASH_WaitForWriteEnd(cs_pin);

    return ret;
}
/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @param  None
 * @retval None
 */
static void sFLASH_WaitForWriteEnd(CS_PIN cs_pin)
{
    uint8_t flashstatus = 0;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW(cs_pin);

    /*!< Send "Read Status Register" instruction */
    sFLASH_SendByte(sFLASH_CMD_RDSR, NULL);

    /*!< Loop as long as the memory is busy with a write cycle */
    do
    {
        /*!< Send a dummy byte to generate the clock needed by the FLASH
         and put the value of the status register in FLASH_Status variable */
        sFLASH_SendByte(sFLASH_DUMMY_BYTE, &flashstatus);
    } while ((flashstatus & sFLASH_WIP_FLAG) == SET); /* Write in progress */

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH(cs_pin);
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @param  None
 * @retval None
 */
static void sFLASH_LowLevel_Init(void)
{
    /** Enable the SPI clock */
    __HAL_RCC_SPI6_CLK_ENABLE()
    ;

    /** SPI configuration */
#if defined (USE_HAL_DRV)

    uint8_t ret = 0;

    sSpiHandle.Instance = SPI6;
    sSpiHandle.Init.Direction = SPI_DIRECTION_2LINES; //SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    sSpiHandle.Init.Mode = SPI_MODE_MASTER;//SPI_Mode = SPI_Mode_Master;
    sSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;//SPI_DataSize = SPI_DataSize_8b;
    sSpiHandle.Init.CLKPolarity = SPI_POLARITY_HIGH;//SPI_CPOL = SPI_CPOL_High;
    sSpiHandle.Init.CLKPhase = SPI_PHASE_2EDGE;//SPI_CPHA = SPI_CPHA_2Edge;
    sSpiHandle.Init.NSS = SPI_NSS_SOFT;//SPI_NSS = SPI_NSS_Soft;
    sSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;//SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;   // 84MHz (APB2) / 4 = 21MHz

    sSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;//PI_FirstBit = SPI_FirstBit_MSB;
    sSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    sSpiHandle.Init.CRCPolynomial = 7;//SPI_CRCPolynomial = 7;
    sSpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;

    //SPI_Init(sFLASH_SPI, &SPI_InitStructure);
    ret = HAL_SPI_Init(&sSpiHandle);

    // HAL_SPI_Transmit_xx () will enable the SPI
    //SPI_Cmd(sFLASH_SPI, ENABLE);  // Enable the sFLASH_SPI

#else

    SPI_InitTypeDef SPI_InitStructure;

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // 90MHz (APB2) / 4 = 22.5MHz

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(sFLASH_SPI, &SPI_InitStructure);

    /*!< Enable the sFLASH_SPI  */
    SPI_Cmd(sFLASH_SPI, ENABLE);

#endif

}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param  BufferLength: buffer's length
 * @retval PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    FAILED = 0, PASSED = !FAILED
} TestStatus;

static TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2,
        uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}

/*****************************************************************
 ** Function name     : flash_block_erase
 ** Descriptions      : Block erase
 ** Input parameters  : addr -- block number 0,1,2.....
 ** Output parameters : None
 ** Returned value    : The operation result. 0 -- sucess, 1 -- false
 ******************************************************************/
uint8_t flash_block_erase(uint32_t lb)
{
    CS_PIN cs = 0;
    uint32_t addr;
    uint8_t cmd[4] =
    { 0 };

    if (lb >= FLASH_BLOCK_COUNT)
    {
        return 1;
    }

    if (lb < FLASH_BLOCKS_PER_CHIP)
    {
        cs = CS1;
        addr = lb * FLASH_BLOCK_SIZE;
    }
    else
    {
        cs = CS2;
        addr = (lb - FLASH_BLOCKS_PER_CHIP) * FLASH_BLOCK_SIZE;
    }

    /*send address command.*/
    cmd[0] = FLASH_CMD_BLOCK_ERASE_4K;
    cmd[1] = ((addr & 0xFF0000) >> 16);
    cmd[2] = ((addr & 0x00FF00) >> 8);
    cmd[3] = (addr & 0x0000FF);

    osMutexWait(spi_mutex, osWaitForever);
    sFLASH_WriteEnable(cs);

    CS_ON(cs);

#if defined (USE_HAL_DRV)
    HAL_SPI_Transmit(&sSpiHandle, cmd, 4, 4 * TIMEOUT_PER_BYTE);
#else
    int i;
    for (i = 0; i < 4; i++)
    {
        sFLASH_SendByte(cmd[i], NULL);
    }
#endif		

    CS_OFF(cs);

    sFLASH_WaitForWriteEnd(cs);
    osMutexRelease(spi_mutex);

    return 0;
}

/*
 Buffer
 Pointer to the byte array to store the read data.
 The buffer size of number of bytes to be read, sector size * sector count, is required.
 Note that the memory address specified by FatFs is not that always aligned to word boundary.
 If the hardware does not support misaligned data transfer, it must be solved in this function.

 page_num
 Specifies the start page number.

 page_cnt
 Specifies number of pages to read..
 */
static uint8_t read_pages_chip(uint8_t* buf, uint32_t page_num,
        uint8_t page_cnt, CS_PIN cs)
{
    uint32_t addr = 0;
    uint32_t request_bytes = 0;

    if ((buf == NULL) || (page_cnt == 0)
            || ((page_num + page_cnt) > FLASH_PAGES_PER_CHIP))
    {
        return 1;
    }

    addr = (page_num * FLASH_PAGE_SIZE);
    request_bytes = (page_cnt * FLASH_PAGE_SIZE);

    SpiFlash_ReadWrite(NULL, buf, addr, request_bytes, cs);

    return 0;
}

/***************************************************************
 ** Function name     : write_page, program only one single page.
 ** Descriptions      : Write flash memory , just in one page memory
 ** Input parameters  : page_num    -- the start address to write
 **                         : buf      --      the buffer to write the data
 **                         : cs   --         chip select.
 ** Returned value    : The operation result. 0 -- sucess, 1 -- false
 ***************************************************************/
static uint8_t write_page(const uint8_t *buf, uint32_t page_num, CS_PIN cs)
{
    uint32_t addr = 0;

    if ((buf == 0) || (page_num >= FLASH_PAGES_PER_CHIP))
    {
        return 1;
    }

    addr = (page_num * FLASH_PAGE_SIZE);

    SpiFlash_ReadWrite(buf, NULL, addr, FLASH_PAGE_SIZE, cs);

    return 0;
}

/***************************************************************
 ** Function name     : write_sector, program only one single sector.
 ** Descriptions      : Write flash memory , just in one page memory
 ** Input parameters  : sec_num    -- the start address to write
 **                         : buf      --      the buffer to write the data
 ** Returned value    : The operation result. 1 -- sucess, 0 -- false
 ***************************************************************/
static uint8_t write_sector(const uint8_t* buf, uint32_t sec_num)
{
    uint8_t i;
    CS_PIN cs;
    uint32_t page_num = 0;
    uint32_t page_cnt = 0;

    if (FLASH_SECTOR_SIZE == FLASH_BLOCK_SIZE)
    {
        flash_block_erase(sec_num);
    }

    page_num = (sec_num << FLASH_N_SECTOR_SIZE);
    page_cnt = (1UL << FLASH_N_SECTOR_SIZE);

    if (page_num < FLASH_PAGES_PER_CHIP)
    {
        cs = CS1;
    }
    else
    {
        cs = CS2;
        page_num -= FLASH_PAGES_PER_CHIP;
    }

    for (i = 0; i < page_cnt; i++)
    {
        if (write_page(buf + (i * FLASH_PAGE_SIZE), (page_num + i), cs))
        {
            return 1;
        }
    }

    return 0;
}

/*
 Buffer
 Pointer to the byte array to store the read data.
 The buffer size of number of bytes to be read, sector size * sector count, is required.
 Note that the memory address specified by FatFs is not that always aligned to word boundary.
 If the hardware does not support misaligned data transfer, it must be solved in this function.

 SectorNumber
 Specifies the start sector number in logical block address (LBA).

 SectorCount
 Specifies number of sectors to read. The value can be 1 to 128.
 */
static uint8_t read_sectors_chip(uint8_t* buf, uint32_t sec_num,
        uint8_t sec_cnt, CS_PIN cs)
{
    uint8_t result;

    if ((buf == 0 || sec_num > FLASH_SECTORS_PER_CHIP || sec_cnt == 0))
    {
        return 1;
    }

    result = read_pages_chip(buf, (sec_num << FLASH_N_SECTOR_SIZE),
            (sec_cnt << FLASH_N_SECTOR_SIZE), cs);

    return result;
}

uint8_t write_sector_small_page(const uint8_t* buf, uint32_t sec_num,
        uint32_t page)
{
    uint8_t result;
    uint8_t i;
    CS_PIN cs = CS1;
    uint32_t pageAdr = 0;
    uint32_t page_cnt = 0;

    if ((buf == 0 || sec_num > FLASH_SECTORS_PER_CHIP))
    {
        return 1;
    }

    pageAdr = sec_num * SECTOR_SZ + page;

    if (pageAdr < FLASH_CAPACITY)
    {
        cs = CS1;
    }
    else
    {
        cs = CS2;
        pageAdr -= FLASH_CAPACITY;
    }

    result = SpiFlash_ReadWrite(buf, NULL, pageAdr,
    FLASH_SMALL_PAGE_SIZE, cs);

    return result;
}

uint8_t read_sector_small_page(uint8_t* buf, uint32_t sec_num, uint32_t page)
{
    uint8_t result;
    uint8_t i;
    CS_PIN cs = CS1;

    uint32_t pageAdr = 0;
    uint32_t page_cnt = 0;

    if ((buf == 0 || sec_num > FLASH_SECTORS_PER_CHIP))
    {
        return 1;
    }

    pageAdr = sec_num * SECTOR_SZ + page;

    if (pageAdr < FLASH_CAPACITY)
    {
        cs = CS1;
    }
    else
    {
        cs = CS2;
        pageAdr -= FLASH_CAPACITY;
    }

    result = SpiFlash_ReadWrite(NULL, buf, pageAdr,
    FLASH_PAGE_SIZE, cs);

    return result;
}

uint8_t read_sector_page(uint8_t* buf, uint32_t sec_num, uint32_t page)
{
    uint8_t result;
    uint8_t i;
    CS_PIN cs;
    uint32_t page_num = 0;
    uint32_t page_cnt = 0;

    if ((buf == 0 || sec_num > FLASH_SECTORS_PER_CHIP))
    {
        return 1;
    }

    page_num = (sec_num << FLASH_N_SECTOR_SIZE) + page / FLASH_PAGE_SIZE;

    if (page_num < FLASH_PAGES_PER_CHIP)
    {
        cs = CS1;
    }
    else
    {
        cs = CS2;
        page_num -= FLASH_PAGES_PER_CHIP;
    }

    result = SpiFlash_ReadWrite(NULL, buf, page_num * FLASH_PAGE_SIZE,
    FLASH_PAGE_SIZE, cs);

    return result;
}

uint8_t write_sector_page(const uint8_t* buf, uint32_t sec_num, uint32_t page)
{
    uint8_t result;
    uint8_t i;
    CS_PIN cs;
    uint32_t page_num = 0;
    uint32_t page_cnt = 0;

    if ((buf == 0 || sec_num > FLASH_SECTORS_PER_CHIP))
    {
        return 1;
    }

    page_num = (sec_num << FLASH_N_SECTOR_SIZE) + page / FLASH_PAGE_SIZE;

    if (page_num < FLASH_PAGES_PER_CHIP)
    {
        cs = CS1;
    }
    else
    {
        cs = CS2;
        page_num -= FLASH_PAGES_PER_CHIP;
    }

    result = SpiFlash_ReadWrite(buf, NULL, (page_num * FLASH_PAGE_SIZE),
    FLASH_PAGE_SIZE, cs);

    return result;
}

uint8_t flash_read_sectors(uint8_t* buf, uint32_t sec_num, uint8_t sec_cnt)
{
    if ((buf == NULL) || (sec_cnt == 0)
            || ((sec_num + sec_cnt) > flash_get_sector_count()))
    {
        return 1;
    }

    if (sec_num >= FLASH_SECTORS_PER_CHIP)/*CS2 only*/
    {
        return read_sectors_chip(buf, (sec_num - FLASH_SECTORS_PER_CHIP),
                sec_cnt, CS2);
    }
    else /*Need read data from CS0*/
    {
        if ((sec_num + sec_cnt) <= FLASH_SECTORS_PER_CHIP) /*CS1 only*/
        {
            return read_sectors_chip(buf, sec_num, sec_cnt, CS1);
        }
        else /*CS1 & CS2*/
        {
            uint32_t cs1_cnt = 0;
            uint32_t cs2_cnt = 0;

            cs1_cnt = FLASH_SECTORS_PER_CHIP - sec_num;
            cs2_cnt = sec_cnt - cs1_cnt;

            /*Read part1 from CS1*/
            if (read_sectors_chip(buf, sec_num, cs1_cnt, CS1))
            {
                return 1;
            }

            /*Read part 2 from CS2*/
            if (read_sectors_chip((buf + (cs1_cnt * flash_get_sector_size())),
                    0 /*start from sector 0*/, cs2_cnt, CS2))
            {
                return 1;
            }

            return 0;
        }
    }
}

/*
 Buffer
 Pointer to the byte array to be written.
 Note that the memory address specified by FatFs is not that always aligned to word boundary.
 If the hardware does not support misaligned data transfer, it must be solved in this function.

 SectorNumber
 Specifies the start sector number in logical block address (LBA).

 SectorCount
 Specifies the number of sectors to write. The value can be 1 to 128.
 */
uint8_t flash_write_sectors(const uint8_t* buf, uint32_t sec_num,
        uint8_t sec_cnt)
{
    uint8_t i;

    if ((buf == NULL) || (sec_cnt == 0)
            || ((sec_num + sec_cnt) > flash_get_sector_count()))
    {
        return 1;
    }

    for (i = 0; i < sec_cnt; i++)
    {
        if (write_sector((buf + (i * (flash_get_sector_size()))),
                (sec_num + i)))
        {
            /*Error : Write Fail*/
            return 1;
        }
    }

    return 0;
}

/******************************************************
 ** Function name     : flash_sectors_erases
 ** Descriptions      : Erase the selected flash
 ** Input parameters  : startSec -- start sector number
 **                     endSec   -- end sector number
 ** Output parameters : None
 ** Returned value    : The operation result. 0 -- sucess, 1 -- false
 ********************************************************/
uint8_t flash_sectors_erase(uint32_t start_sec, uint32_t end_sec)
{
    uint32_t i;
    uint32_t start_block = 0;
    uint32_t end_block = 0;

    if (end_sec < start_sec)
        return 1;
    start_block = (uint32_t) (start_sec / FLASH_N_BLOCK_SIZE);
    end_block = (uint32_t) (end_sec / FLASH_N_BLOCK_SIZE);

    for (i = start_block; i <= end_block; i++)
    {
        flash_block_erase(i);
    }

    return 0;
}

// End of file
