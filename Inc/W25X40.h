/*
 * W25X40.h
 *
 *  Created on: Feb 12, 2023
 *      Author: Tyler
 */



#ifndef INC_W25X40_H_
#define INC_W25X40_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

//Read commands
#define W25X40_Read_Data 0x03
//Program commands
#define W25X40_Page_Erase 0x81
#define W25X40_Sector_Erase 0x20
#define W25X40_Block_Erase_32k 0x52
#define W25X40_Byte_Page_Prog 0x02
#define W25X40_Seq_Prog = 0xAD
#define W25X40_Dual_Byte_Page_Prog 0xA2
//Protection Commands
#define W25X40 = 0x06;
#define W25X40_Write_DISABLE 0x04
#define W25X40_Protect_Sector 0x36
#define W25X40_Unprotect_Sector 0x39
#define W25X40_Read_Protection_Sector 0x3C
//Security Commands
#define W25X40_Prog_OTP_Security_Reg 0x9B
#define W25X40_Read_OTP_Security_Reg 0x77
//Status Register Commands
#define W25X40_Read_Status_Reg 0x05
#define W25X40_Active_Status_Int 0x25
#define W25X40_Write_Status_Reg_1 0x01
#define W25X40_Write_Status_Reg_2 0x31
//Misc Commands
#define W25X40_Reset = 0xF0
#define W25X40_Read_Manufacture_Dev_ID 0x9F
#define W25X40_Deep_Power_Down 0xB9
#define W25X40_Resume_From_Deep_Power_Down 0xAB
#define W25X40_Ultra_Deep_Power_Down 0x79

//Memory Defines
#define W25X40_PAGE_SIZE    256
#define W25X40_SECTOR_SIZE  4096

//Mask Defines
#define W25X40_WIP_MASK 0b00000001
#define W25X40_WEL_MASK 0b00000010


//Function Prototypes
void W25X40_Manu_ID_Check(SPI_HandleTypeDef *spi_instance);
void W25X40_WEL_Check(SPI_HandleTypeDef *spi_instance);
void W25X40_WriteEnable(SPI_HandleTypeDef *spi_instance);
void W25X40_WriteDisable(SPI_HandleTypeDef *spi_instance);
void W25X40_ReadStatus(SPI_HandleTypeDef *spi_instance);
void W25X40_SectorErase(SPI_HandleTypeDef *hspi, uint32_t sector_address);
void W25X40_Busy_Check(SPI_HandleTypeDef *hspi);
void W25X40_ReadData(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint32_t size);
void W25X40_PageProgram(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint32_t size);
void W25X40_Set_Status(SPI_HandleTypeDef *spi_instance);
void W25X40_Chip_Erase(SPI_HandleTypeDef *spi_instance);
void W25X40_Multi_Write_Data(SPI_HandleTypeDef *spi_instance, uint32_t address, uint8_t *data, uint32_t len);
void W25X40_PageProgram2(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint32_t size);
void W25X40_Directory_Read(SPI_HandleTypeDef *hspi);
void W25X40_Entry_Delete(SPI_HandleTypeDef *spi_instance, uint8_t num_Entry);
void W25X40_Available_MEM(SPI_HandleTypeDef *hspi);
void W25X40_WriteString(char *str);
void W25X40_Print_Title(SPI_HandleTypeDef *hspi, uint32_t address);
void W25X40_Available_Entries(SPI_HandleTypeDef *hspi);
void W25X40_Store_Data(SPI_HandleTypeDef *hspi, uint8_t line_Num);
void W25X40_Read_Data_Entry(SPI_HandleTypeDef *hspi,uint8_t num_Entry);//add non global data pass

#endif /* INC_W25X40_H_ */
