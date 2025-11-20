/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "ff.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* --- External SPI and CS pin from CubeMX --- */
extern SPI_HandleTypeDef hspi2;

#ifndef SD_CS_GPIO_Port
#error "Define SD_CS_GPIO_Port and SD_CS_Pin in gpio.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Card type flags (CardType) */
#define CT_MMC   0x01U    /* MMC ver 3 */
#define CT_SD1   0x02U    /* SD ver 1 */
#define CT_SD2   0x04U    /* SD ver 2 */
#define CT_SDC   (CT_SD1 | CT_SD2) /* SD */
#define CT_BLOCK 0x08U    /* Block addressing */

/* Command definitions (SPI mode) */
#define CMD0     (0U)          /* GO_IDLE_STATE */
#define CMD1     (1U)          /* SEND_OP_COND (MMC) */
#define CMD8     (8U)          /* SEND_IF_COND */
#define CMD9     (9U)          /* SEND_CSD */
#define CMD10    (10U)         /* SEND_CID */
#define CMD12    (12U)         /* STOP_TRANSMISSION */
#define CMD16    (16U)         /* SET_BLOCKLEN */
#define CMD17    (17U)         /* READ_SINGLE_BLOCK */
#define CMD18    (18U)         /* READ_MULTIPLE_BLOCK */
#define CMD23    (23U)         /* SET_BLOCK_COUNT (MMC) */
#define CMD24    (24U)         /* WRITE_SINGLE_BLOCK */
#define CMD25    (25U)         /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (41U)         /* SEND_OP_COND (ACMD) */
#define CMD55    (55U)         /* APP_CMD */
#define CMD58    (58U)         /* READ_OCR */
#define ACMD23   (0x80U | 23U) /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define ACMD41   (0x80U | 41U) /* SD_SEND_OP_COND (SDC) */

/* Timeouts (ms) */
#define SD_SPI_TIMEOUT     100U
#define SD_INIT_TIMEOUT   1000U
#define SD_READ_TIMEOUT    100U
#define SD_WRITE_TIMEOUT   250U

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
/* Card type */
static BYTE CardType = 0;

/* Low-level static helpers --------------------------------------------------*/
static void  SD_Select(void);
static void  SD_Deselect(void);
static BYTE  SD_SPI_TxRx(BYTE data);
static void  SD_SendDummyClocks(UINT n);
static int   SD_WaitReady(UINT timeout);
static BYTE  SD_SendCmd(BYTE cmd, DWORD arg);
static int   SD_RecvDataBlock(BYTE *buff, UINT btr);
static int   SD_SendDataBlock(const BYTE *buff, BYTE token);
static DRESULT SD_ReadSectors(BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
static DRESULT SD_WriteSectors(const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */

/* -------------------------------------------------------------------------- */
/*                       Static helper implementations                        */
/* -------------------------------------------------------------------------- */

static void SD_Select(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void SD_Deselect(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  (void)SD_SPI_TxRx(0xFFU); /* One extra clock after CS high */
}

static BYTE SD_SPI_TxRx(BYTE data)
{
  uint8_t tx = data;
  uint8_t rx = 0xFFU;

  if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U, SD_SPI_TIMEOUT) != HAL_OK)
  {
    /* In case of SPI error return idle line value */
    return 0xFFU;
  }
  return (BYTE)rx;
}

static void SD_SendDummyClocks(UINT n)
{
  while (n--)
  {
    (void)SD_SPI_TxRx(0xFFU);
  }
}

static int SD_WaitReady(UINT timeout)
{
  uint32_t start = HAL_GetTick();
  BYTE    resp;

  do
  {
    resp = SD_SPI_TxRx(0xFFU);
    if (resp == 0xFFU)
    {
      return 0; /* Ready */
    }
  } while ((HAL_GetTick() - start) < timeout);

  return 1; /* Timeout */
}

/* Send a command packet to the card */
static BYTE SD_SendCmd(BYTE cmd, DWORD arg)
{
  BYTE res;
  BYTE buf[6];
  UINT n;

  /* ACMD<n> is the command sequence of CMD55-CMD<n> */
  if (cmd & 0x80U)
  {
    cmd &= 0x7FU;
    res = SD_SendCmd(CMD55, 0U);
    if (res > 1U)
    {
      return res;
    }
  }

  SD_Deselect();
  (void)SD_SPI_TxRx(0xFFU);
  SD_Select();

  if (SD_WaitReady(SD_INIT_TIMEOUT) != 0)
  {
    SD_Deselect();
    return 0xFFU;
  }

  /* Command packet */
  buf[0] = (BYTE)(0x40U | cmd);
  buf[1] = (BYTE)(arg >> 24);
  buf[2] = (BYTE)(arg >> 16);
  buf[3] = (BYTE)(arg >> 8);
  buf[4] = (BYTE)arg;

  /* Default CRC, special for CMD0 and CMD8 */
  BYTE crc = 0x01U;
  if (cmd == CMD0)
  {
    crc = 0x95U; /* Valid CRC for CMD0 + arg 0 */
  }
  else if (cmd == CMD8)
  {
    crc = 0x87U; /* Valid CRC for CMD8 + arg 0x1AA */
  }
  buf[5] = crc;

  for (n = 0U; n < 6U; n++)
  {
    (void)SD_SPI_TxRx(buf[n]);
  }

  /* Receive response (expect MSB=0) */
  n = 10U;
  do
  {
    res = SD_SPI_TxRx(0xFFU);
  } while ((res & 0x80U) && --n);

  return res;
}

/* Receive a data block from the card */
static int SD_RecvDataBlock(BYTE *buff, UINT btr)
{
  BYTE token;
  uint32_t start = HAL_GetTick();

  /* Wait for data token 0xFE */
  do
  {
    token = SD_SPI_TxRx(0xFFU);
    if (token == 0xFEU)
    {
      break;
    }
  } while ((HAL_GetTick() - start) < SD_READ_TIMEOUT);

  if (token != 0xFEU)
  {
    return 0; /* Invalid data token */
  }

  /* Receive data block */
  for (UINT i = 0U; i < btr; i++)
  {
    buff[i] = SD_SPI_TxRx(0xFFU);
  }

  /* Discard CRC */
  (void)SD_SPI_TxRx(0xFFU);
  (void)SD_SPI_TxRx(0xFFU);

  return 1;
}

/* Send a data block to the card */
static int SD_SendDataBlock(const BYTE *buff, BYTE token)
{
  BYTE resp;

  if (SD_WaitReady(SD_WRITE_TIMEOUT) != 0)
  {
    return 0;
  }

  (void)SD_SPI_TxRx(token);

  if (token != 0xFDU) /* 0xFD = Stop Tran token */
  {
    /* Send 512-byte block */
    for (UINT i = 0U; i < 512U; i++)
    {
      (void)SD_SPI_TxRx(buff[i]);
    }

    /* Dummy CRC */
    (void)SD_SPI_TxRx(0xFFU);
    (void)SD_SPI_TxRx(0xFFU);

    /* Data response */
    resp = SD_SPI_TxRx(0xFFU);
    if ((resp & 0x1FU) != 0x05U)
    {
      return 0; /* Data not accepted */
    }
  }

  return 1;
}

/* Read sectors (internal helper) */
static DRESULT SD_ReadSectors(BYTE *buff, DWORD sector, UINT count)
{
  if (!(CardType & CT_BLOCK))
  {
    sector *= 512U; /* Convert to byte address for SDSC / MMC */
  }

  if (count == 1U)
  {
    if (SD_SendCmd(CMD17, sector) != 0U) /* READ_SINGLE_BLOCK */
    {
      SD_Deselect();
      return RES_ERROR;
    }

    if (!SD_RecvDataBlock(buff, 512U))
    {
      SD_Deselect();
      return RES_ERROR;
    }

    count = 0U;
  }
  else
  {
    if (SD_SendCmd(CMD18, sector) != 0U) /* READ_MULTIPLE_BLOCK */
    {
      SD_Deselect();
      return RES_ERROR;
    }

    do
    {
      if (!SD_RecvDataBlock(buff, 512U))
      {
        break;
      }
      buff += 512U;
    } while (--count);

    (void)SD_SendCmd(CMD12, 0U); /* STOP_TRANSMISSION */
  }

  SD_Deselect();
  (void)SD_SPI_TxRx(0xFFU);

  return (count == 0U) ? RES_OK : RES_ERROR;
}

#if _USE_WRITE == 1
/* Write sectors (internal helper) */
static DRESULT SD_WriteSectors(const BYTE *buff, DWORD sector, UINT count)
{
  if (!(CardType & CT_BLOCK))
  {
    sector *= 512U; /* Convert to byte address */
  }

  if (count == 1U)
  {
    if (SD_SendCmd(CMD24, sector) != 0U) /* WRITE_SINGLE_BLOCK */
    {
      SD_Deselect();
      return RES_ERROR;
    }

    if (!SD_SendDataBlock(buff, 0xFEU))
    {
      SD_Deselect();
      return RES_ERROR;
    }

    count = 0U;
  }
  else
  {
    if (CardType & CT_SDC)
    {
      /* Optional: inform about number of blocks before multi-block write */
      (void)SD_SendCmd(ACMD23, count);
    }

    if (SD_SendCmd(CMD25, sector) != 0U) /* WRITE_MULTIPLE_BLOCK */
    {
      SD_Deselect();
      return RES_ERROR;
    }

    do
    {
      if (!SD_SendDataBlock(buff, 0xFCU)) /* Data token for multi-block */
      {
        break;
      }
      buff += 512U;
    } while (--count);

    /* STOP_TRAN token */
    if (!SD_SendDataBlock(NULL, 0xFDU))
    {
      SD_Deselect();
      return RES_ERROR;
    }
  }

  SD_Deselect();
  (void)SD_SPI_TxRx(0xFFU);

  return (count == 0U) ? RES_OK : RES_ERROR;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  BYTE    ty    = 0U;
  BYTE    ocr[4];
  BYTE    res;
  uint32_t start;

  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  if ((Stat & STA_NOINIT) == 0U)
  {
    return Stat; /* Уже ініціалізовано */
  }

  CardType = 0U;

  SD_Deselect();
  SD_SendDummyClocks(10U); /* >74 тактів з CS=H */

  res = SD_SendCmd(CMD0, 0U); /* Перехід у IDLE */

  if (res == 1U)
  {
    /* Перевірка підтримки напруги та SD v2 */
    if (SD_SendCmd(CMD8, 0x1AAU) == 1U)
    {
      for (UINT i = 0U; i < 4U; i++)
      {
        ocr[i] = SD_SPI_TxRx(0xFFU);
      }

      if ((ocr[2] == 0x01U) && (ocr[3] == 0xAAU))
      {
        /* Чекаємо виходу з IDLE з HCS=1 */
        start = HAL_GetTick();
        do
        {
          res = SD_SendCmd(ACMD41, 1UL << 30);
        } while (res && ((HAL_GetTick() - start) < SD_INIT_TIMEOUT));

        if ((res == 0U) && (SD_SendCmd(CMD58, 0U) == 0U))
        {
          for (UINT i = 0U; i < 4U; i++)
          {
            ocr[i] = SD_SPI_TxRx(0xFFU);
          }
          ty = (ocr[0] & 0x40U) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
        }
      }
    }
    else
    {
      /* SD v1 або MMC */
      BYTE cmd;

      if (SD_SendCmd(ACMD41, 0U) <= 1U)
      {
        ty  = CT_SD1;
        cmd = ACMD41;
      }
      else
      {
        ty  = CT_MMC;
        cmd = CMD1;
      }

      start = HAL_GetTick();
      do
      {
        res = SD_SendCmd(cmd, 0U);
      } while (res && ((HAL_GetTick() - start) < SD_INIT_TIMEOUT));

      if ((res != 0U) || (SD_SendCmd(CMD16, 512U) != 0U))
      {
        ty = 0U;
      }
    }
  }

  CardType = ty;
  SD_Deselect();
  (void)SD_SPI_TxRx(0xFFU);

  if (ty != 0U)
  {
    Stat &= ~STA_NOINIT;
  }
  else
  {
    Stat = STA_NOINIT;
  }

  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  if (pdrv != 0U)
  {
    return STA_NOINIT;
  }

  return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  if ((pdrv != 0U) || (count == 0U))
  {
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT)
  {
    return RES_NOTRDY;
  }

  return SD_ReadSectors(buff, sector, count);
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  if ((pdrv != 0U) || (count == 0U))
  {
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT)
  {
    return RES_NOTRDY;
  }

  /* За потреби тут можна врахувати фізичний write-protect пін і встановити STA_PROTECT */

  return SD_WriteSectors(buff, sector, count);
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  DRESULT res = RES_ERROR;

  if (pdrv != 0U)
  {
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT)
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
      if (SD_WaitReady(SD_WRITE_TIMEOUT) == 0)
      {
        res = RES_OK;
      }
      SD_Deselect();
      (void)SD_SPI_TxRx(0xFFU);
      break;

    case GET_SECTOR_COUNT:
    {
      BYTE  csd[16];
      DWORD csz;

      if ((SD_SendCmd(CMD9, 0U) == 0U) && SD_RecvDataBlock(csd, 16U))
      {
        if ((csd[0] >> 6) == 1U)
        {
          /* CSD версії 2.0 (SDHC/SDXC) */
          csz  = (DWORD)csd[9];
          csz |= (DWORD)csd[8] << 8;
          csz |= (DWORD)(csd[7] & 0x3FU) << 16;
          csz  = (csz + 1U) << 10; /* Кількість секторів по 512 байт */
          *(DWORD *)buff = csz;
        }
        else
        {
          /* CSD версії 1.x / MMC */
          BYTE n;
          csz  = (DWORD)(csd[8] & 0xC0) >> 6;
          csz |= (DWORD)csd[7] << 2;
          csz |= (DWORD)(csd[6] & 0x03U) << 10;
          n  = (BYTE)((csd[5] & 0x0FU)
               + ((csd[10] & 0x80U) >> 7)
               + ((csd[9] & 0x03U) << 1)
               + 2U);
          *(DWORD *)buff = (csz + 1U) << (n - 9U);
        }
        res = RES_OK;
      }
      SD_Deselect();
      (void)SD_SPI_TxRx(0xFFU);
      break;
    }

    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512U;
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE:
      /* Розмір стираного блоку в секторах (мінімум 1, якщо точно невідомо) */
      *(DWORD *)buff = 1U;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
      break;
  }

  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

