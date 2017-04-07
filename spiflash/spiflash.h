/*--------------------------------------------------------------------------
 * spiflash.h -
 *
 * Author: 185275258 (QQ Group)
 *
 *--------------------------------------------------------------------------*/

#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

/** M25P SPI Flash supported commands */
#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_UNSE		  0x39  /* Un-protect sector */
#define sFLASH_CMD_RDSEPR  	  0x3C  /* Read sector protect register */
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /* Read identification */
#define sFLASH_CMD_SE             0xD8  /* Sector (64k bytes) Erase instruction */
#define sFLASH_CMD_BE             0xC7  /* Bulk Erase instruction */

/** Command List */
#define FLASH_CMD_READ0 		sFLASH_CMD_READ
#define FLASH_CMD_READ1   		(0x0B)
#define FLASH_CMD_READ2   		(0x1B)
#define FLASH_CMD_READ    		FLASH_CMD_READ2
#define FLASH_CMD_WRITE_ENABLE		sFLASH_CMD_WREN
#define FLASH_CMD_WRITE_DISABLE 	0x04
#define FLASH_CMD_WRITE           	sFLASH_CMD_WRITE
#define FLASH_CMD_BLOCK_ERASE_4K  	0x20
#define FLASH_CMD_BLOCK_ERASE_32K  	0x52
#define FLASH_CMD_BLOCK_ERASE_64K  	sFLASH_CMD_SE
#define FLASH_CMD_CHIP_ERASE0   	0x60
#define FLASH_CMD_CHIP_ERASE1     	sFLASH_CMD_BE
#define FLASH_CMD_READ_STAT_REG    	0x05
#define FLASH_CMD_WRITE_STATUS_BYTE1	sFLASH_CMD_WRSR
#define FLASH_CMD_WRITE_STATUS_BYTE2 	0x31
#define FLASH_CMD_RESET           	0xF0
#define FLASH_CMD_READ_MANU_n_DevID  	sFLASH_CMD_RDID

#define sFLASH_WEL_FLAG			0x02  	/* Write enable flag */
#define sFLASH_WIP_FLAG           	0x01  	/* Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         	0xA5
#define sFLASH_SPI_PAGESIZE       	0x100 	/* 256 bytes */
#define sFLASH_SPI_SECTORSIZE	  	0x10000	/* 64k bytes*/
#define sFLASH_M25P128_ID         	0x202018
#define sFLASH_M25P64_ID         	0x202017
#define sFLASH_AT25DF641_ID		0x1F4800

typedef enum
{
    CS1 = 0, CS2 = 1
} CS_PIN;

/* Select sFLASH: Chip Select pin low */
#define sFLASH_CS_LOW(a) ((a == CS1) ? (Dev_Gpio_DOut_TurnOn(e_Dev_Dout_SpiFlash_CS1)) : (Dev_Gpio_DOut_TurnOn(e_Dev_Dout_SpiFlash_CS2)))
#define CS_ON sFLASH_CS_LOW

/* Deselect sFLASH: Chip Select pin high */
#define sFLASH_CS_HIGH(a) ((a == CS1) ? (Dev_Gpio_DOut_TurnOff(e_Dev_Dout_SpiFlash_CS1)) : (Dev_Gpio_DOut_TurnOff(e_Dev_Dout_SpiFlash_CS2)))
#define CS_OFF sFLASH_CS_HIGH

/*SPI Flash devices list*/
enum
{
    AT25DF641A, AT25DF321A, RAMDISK
};

/*define the flash device */

#define DISK_TYPE AT25DF641A
#define FLASH_SECTORS_RESERVED	128					/* 32*4 = 128K */
#define FLASH_RVD_SPACE  	(FLASH_SECTORS_RESERVED*SECTOR_SZ)	/*200Kbytes*/	
#if (DISK_TYPE == AT25DF641A)
#define SECTOR_SZ  		4096
#define FLASH_SMALL_PAGE_SIZE   (64UL)		/* The size of page , 64bytes */
#define FLASH_PAGE_SIZE        	(256UL)		/* The size of page , 256Bytes */
#define FLASH_N_SECTOR_SIZE	(4) 		/* 16 pages-per-sector */
#define FLASH_CAPACITY          (0x800000) 	/*  8Mbytes */
#define FLASH_SECTOR_SIZE       ((FLASH_PAGE_SIZE)<<(FLASH_N_SECTOR_SIZE)) /* Size of sector, 4kB */
#define FLASH_BLOCK_SIZE        ((FLASH_SECTOR_SIZE)<<0)	    	/* The size of block, 4kB */
#define FLASH_BLOCK_COUNT       (FLASH_CAPACITY/FLASH_BLOCK_SIZE)
#define FLASH_SECTOR_COUNT      (FLASH_CAPACITY/FLASH_SECTOR_SIZE)
#define FLASH_N_BLOCK_SIZE      (FLASH_BLOCK_SIZE/FLASH_SECTOR_SIZE) 	/* 1 Sector-per-Block */
#define FLASH_BLOCKS_PER_CHIP 	(FLASH_BLOCK_COUNT)
#define FLASH_PAGES_PER_CHIP  	(0x8000)
#define FLASH_SECTORS_PER_CHIP 	(FLASH_PAGES_PER_CHIP>>FLASH_N_SECTOR_SIZE)

#elif (DISK_TYPE == W25X32)

#define FLASH_PAGE_SIZE         (256UL)     	/* The size of page  */
#define FLASH_PAGE_COUNT        (0x4000)     	/* The  page  count  */
#define FLASH_N_SECTOR_SIZE     (1)  		/*2 pages-per-sector */
#define FLASH_SECTOR_SIZE       ((FLASH_PAGE_SIZE)<<(FLASH_N_SECTOR_SIZE))
#define FLASH_SECTOR_COUNT      (FLASH_PAGE_COUNT>>FLASH_N_SECTOR_SIZE)
#define FLASH_BLOCK_SIZE        (FLASH_SECTOR_SIZE<<3)	 /* The size of block */
#define FLASH_CAPACITY          (0x400000)
#define FLASH_BLOCK_COUNT       (FLASH_CAPACITY/FLASH_BLOCK_SIZE)
#define FLASH_N_BLOCK_SIZE      (FLASH_BLOCK_SIZE/FLASH_SECTOR_SIZE)

#elif (DISK_TYPE==AT25DF321A)

#define SECTOR_SZ  		4096
#if (SECTOR_SZ==512) 				/* Sector Size:512Bytes */
#define FLASH_PAGE_SIZE         (256UL) 	/* The size of page , 256Bytes */
#define FLASH_N_SECTOR_SIZE     (1) 		/* 2 pages-per-sector */
#define FLASH_CAPACITY          (0x800000)      /*  2*4Mbytes=8Mbytes */
#define FLASH_SECTOR_SIZE       ((FLASH_PAGE_SIZE)<<(FLASH_N_SECTOR_SIZE)) /* Size of sector,512B*/
#define FLASH_BLOCK_SIZE        ((FLASH_SECTOR_SIZE)<<3)	    /* The size of block,4KBytes  */
#define FLASH_BLOCK_COUNT       (FLASH_CAPACITY/FLASH_BLOCK_SIZE)
#define FLASH_SECTOR_COUNT      (FLASH_CAPACITY/FLASH_SECTOR_SIZE)
#define FLASH_N_BLOCK_SIZE      (FLASH_BLOCK_SIZE/FLASH_SECTOR_SIZE) /*8 Sectors-per-Block*/

#define FLASH_BLOCKS_PER_CHIP   (FLASH_BLOCK_COUNT>>1)
#define FLASH_PAGES_PER_CHIP    (0x4000)
#define FLASH_SECTORS_PER_CHIP  (FLASH_PAGES_PER_CHIP>>1)
#define FLASH_CHIP_CAPACITY     (0x400000)

#elif  (SECTOR_SZ==4096) 			/*Sector Size:4096Bytes*/

#define FLASH_PAGE_SIZE         (256UL)      	/* The size of page , 256Bytes*/
#define FLASH_N_SECTOR_SIZE     (4) 		/* 16 pages-per-sector*/
#define FLASH_CAPACITY          (0x800000)      /* 2*4Mbytes=8Mbytes */
#define FLASH_SECTOR_SIZE       ((FLASH_PAGE_SIZE)<<(FLASH_N_SECTOR_SIZE)) /*Size of sector,512B*/
#define FLASH_BLOCK_SIZE        ((FLASH_SECTOR_SIZE)<<0)	    /* The size of block,4KBytes  */
#define FLASH_BLOCK_COUNT       (FLASH_CAPACITY/FLASH_BLOCK_SIZE)
#define FLASH_SECTOR_COUNT      (FLASH_CAPACITY/FLASH_SECTOR_SIZE)
#define FLASH_N_BLOCK_SIZE      (FLASH_BLOCK_SIZE/FLASH_SECTOR_SIZE) /*1 Sector-per-Block*/

#define FLASH_BLOCKS_PER_CHIP   (FLASH_BLOCK_COUNT>>1)
#define FLASH_PAGES_PER_CHIP    (0x4000)
#define FLASH_SECTORS_PER_CHIP  (FLASH_PAGES_PER_CHIP>>FLASH_N_SECTOR_SIZE)
#define FLASH_CHIP_CAPACITY     (0x400000)
#endif /*SECTOR_SZ*/

#elif (DISK_TYPE==RAMDISK)
#define FLASH_SECTOR_SIZE       (4096) /*Size of sector*/
#define FLASH_CAPACITY          (0x800000)
#define FLASH_BLOCK_SIZE        ((FLASH_SECTOR_SIZE)<<0)	    /* The size of block,nBytes  */
#define FLASH_BLOCK_COUNT       (FLASH_CAPACITY/FLASH_BLOCK_SIZE)
#define FLASH_SECTOR_COUNT      (FLASH_CAPACITY/FLASH_SECTOR_SIZE)
#define FLASH_N_BLOCK_SIZE      (FLASH_BLOCK_SIZE/FLASH_SECTOR_SIZE) /* Sectors-per-Block*/
#endif

/******************************************************************
 ** Function name  :flash_read_sectors
 Buffer
 Pointer to the byte array to store the read data.
 The buffer size of number of bytes to be read, sector size * sector count, is required.
 Note that the memory address specified by FatFs is not that always aligned to word boundary.
 If the hardware does not support misaligned data transfer, it must be solved in this function.

 SectorNumber
 Specifies the start sector number in logical block address (LBA).

 SectorCount
 Specifies number of sectors to read. The value can be 1 to 128.

 Return : 0: success, else fail.
 ********************************************************************/
extern uint8_t flash_read_sectors(uint8_t* Buffer, uint32_t sec_num,
        uint8_t sec_cnt);

/******************************************************************
 ** Function name  :flash_write_sectors
 Buffer
 Pointer to the byte array to be written.
 Note that the memory address specified by FatFs is not that always aligned to word boundary.
 If the hardware does not support misaligned data transfer, it must be solved in this function.

 SectorNumber
 Specifies the start sector number in logical block address (LBA).

 SectorCount
 Specifies the number of sectors to write. The value can be 1 to 128.

 Return : 0: success, else fail.
 ********************************************************************/
extern uint8_t flash_write_sectors(const uint8_t* Buffer, uint32_t sec_num,
        uint8_t sec_cnt);

/******************************************************
 ** Function name     : flash_sectors_erases
 ** Descriptions      : Erase the selected flash
 ** Input parameters  : startSec -- start sector	number
 **                     endSec   -- end sector number
 ** Output parameters : None
 ** Returned value    : The operation result. 0 -- sucess, 1 -- false
 ********************************************************/
extern uint8_t flash_sectors_erase(uint32_t start_sec, uint32_t end_sec);

uint8_t flash_block_erase(uint32_t lb);
uint8_t write_sector_page(const uint8_t* buf, uint32_t sec_num, uint32_t page);
uint8_t read_sector_page(uint8_t* buf, uint32_t sec_num, uint32_t page);
uint8_t read_sector_small_page(uint8_t* buf, uint32_t sec_num, uint32_t page);

uint8_t write_sector_small_page(const uint8_t* buf, uint32_t sec_num,
        uint32_t page);
/*inline functions*/

/*Returns sector size of the drive into the WORD variable pointed by Buffer.
 This command is not used in fixed sector size configuration, _MAX_SS is 512*/
static __inline uint32_t flash_get_sector_size(void)
{
    return (FLASH_SECTOR_SIZE);
}

/* Get erase block size in unit of sector (DWORD) */
static __inline uint32_t flash_get_block_size(void)
{
    return (FLASH_BLOCK_SIZE / FLASH_SECTOR_SIZE);
}

/*Get the flash capacity (Bytes)*/
static __inline uint32_t flash_get_capacity(void)
{
    return (FLASH_CAPACITY);
}

/*
 Returns number of available sectors on the drive to determine the volume size to be created
 */
static __inline uint32_t flash_get_sector_count(void)
{
    return (FLASH_SECTOR_COUNT);
}

uint8_t SpiFlash_Init(void);

uint8_t SpiFlash_ReadWrite(const uint8_t *txBuf, uint8_t *rxBuf, 
			   uint32_t addr, uint16_t rwBytes, CS_PIN cs_pin);

uint8_t SpiFlash_ReadID(CS_PIN cs_pin, uint32_t *p_id);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */

