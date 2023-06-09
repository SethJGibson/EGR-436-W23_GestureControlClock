#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "W25X40.h"
#include "main.h"


/*
 * W25X40.c
 *
 *  Created on: Feb 12, 2023
 *      Author: Tyler
 */

uint8_t avail_addr[10]={0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0};
uint32_t mem_used_addr = 0x000000;
uint32_t num_entries_addr = 0x001000;
uint32_t mem_1_addr = 0x002000;
uint32_t mem_2_addr = 0x003000;
uint32_t mem_3_addr = 0x004000;
uint32_t mem_4_addr = 0x005000;
uint32_t mem_5_addr = 0x006000;
uint32_t mem_6_addr = 0x007000;
uint32_t mem_7_addr = 0x008000;
uint32_t mem_8_addr = 0x009000;
uint32_t mem_9_addr = 0x00A000;
uint32_t mem_10_addr= 0x00B000;

uint32_t entry1=0x011000;//addresses holding actual text data
uint32_t entry2=0x012000;
uint32_t entry3=0x013000;
uint32_t entry4=0x014000;
uint32_t entry5=0x015000;
uint32_t entry6=0x016000;
uint32_t entry7=0x017000;
uint32_t entry8=0x018000;
uint32_t entry9=0x019000;
uint32_t entry10=0x01A000;

uint32_t data_storage = 0x002010;
uint8_t num_entries=0x00;
uint8_t string_to_array [256]={0};
uint8_t string_length=0;

uint8_t fullFileBuffer[22][256] = {{0}};

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Manu_ID_Check
 *Description: Reads JEDEC ID from W25X40 and prints the device and manufacture information
 *			   to the STM32 connector COM port using BR 115200
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Manu_ID_Check(SPI_HandleTypeDef *spi_instance)
{
	uint8_t devid_cmd[1] = {0x9F};
	uint8_t devid_res[4];

	HAL_StatusTypeDef res1, res2;

	// bring cs low, transmit and receive data
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	res1 = HAL_SPI_Transmit(spi_instance, devid_cmd, sizeof(devid_cmd), HAL_MAX_DELAY);
	res2 = HAL_SPI_Receive(spi_instance, devid_res, sizeof(devid_res), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//bring cs high to end transmit and recieve

	if ((res1 != HAL_OK) || (res2 != HAL_OK))
	{
		printf("Error\n\r");

	}
	//print device info to ensure proper operation
	printf(
			"Manufacturer ID: 0x%X\r\n"
			"Device ID (ID15-8): 0x%X\r\n"
			"Device ID (ID7-0): 0x%X\r\n"
			"--------\r\n",
			devid_res[0], devid_res[1], devid_res[2]);

}



/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_WEL_Check
 *Description: Checks to make sure the latch is set before exiting.
 *			   Prints to the STM32 connected COM port is the operation is performed properly. THis
 *			   function will loop until the WEL bit is set properly.
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_WEL_Check(SPI_HandleTypeDef *spi_instance)
{
	uint8_t status=0x00;


	uint8_t temp = 0x05;//temp variable with desired register
	// send read status register command
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(spi_instance, &temp, 1, HAL_MAX_DELAY); // send read status register command
	HAL_SPI_Receive(spi_instance, &status, 1, HAL_MAX_DELAY); // receive the status register value
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40
	if((status & W25X40_WEL_MASK) != W25X40_WEL_MASK)
	{
		printf("WEL Disabled\n\r");
	}
	else
	{
		printf("WEL Enabled\n\r");
	}
}


/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_WriteEnable
 *Description: Sets the WEL bit to 1 allowing flash manipulation
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_WriteEnable(SPI_HandleTypeDef *spi_instance)
{
	uint8_t cmd = 0x06;

	// Chip select for the AT25DF041B
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	// Send the Write Enable command
	HAL_SPI_Transmit(spi_instance, &cmd, 1, HAL_MAX_DELAY);

	// Chip deselect for the AT25DF041B
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_WriteEnable
 *Description: Sets the WEL bit to 0 locking device from memory changes
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_WriteDisable(SPI_HandleTypeDef *spi_instance)
{

	uint8_t cmd = 0x04;

	// Chip select for the AT25DF041B
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	// Send the Write Enable command
	HAL_SPI_Transmit(spi_instance, &cmd, 1, HAL_MAX_DELAY);

	// Chip deselect for the AT25DF041B
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

}


/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_ReadStatus
 *Description: Reads status register and prints contents, in hex, to the STM32 connect COM port
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_ReadStatus(SPI_HandleTypeDef *spi_instance)
{
	uint8_t status;
	uint8_t temp = 0x05;//temp variable with desired register

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(spi_instance, &temp, 1, HAL_MAX_DELAY); // send read status register command
	HAL_SPI_Receive(spi_instance, &status, 1, HAL_MAX_DELAY); // receive the status register value
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	printf("--------\n\r""Status Register: 0x%x\n\r""--------\n\r",status);

}



/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_SectorErase
 *Description: Erases sector of memory starting at the provided address
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_SectorErase(SPI_HandleTypeDef *hspi, uint32_t sector_address)
{
	uint8_t command[4];

	// Send write enable command
	W25X40_WriteEnable(hspi);

	// Send sector erase command
	command[0] = 0x20;
	command[1] = (sector_address & 0xFF0000) >> 16;
	command[2] = (sector_address & 0xFF00) >> 8;
	command[3] = (sector_address & 0xFF);

	//pull CS low to initiate data send
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	//Send data
	HAL_SPI_Transmit(hspi, command, 4, 1000);
	//pull cs high to end data stream
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	// Wait for the erase to complete
	W25X40_Busy_Check(hspi);


	//printf("Sector %lx Erase Complete\n\r", sector_address);

}


/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Busy_Check
 *Description: Waits for busy flag to go to zero to know that more instructions can be sent
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Busy_Check(SPI_HandleTypeDef *hspi)
{
	uint8_t status;
	uint8_t temp = 0x05;//temp variable with desired register

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(hspi, &temp, 1, HAL_MAX_DELAY); // send read status register command
	HAL_SPI_Receive(hspi, &status, 1, HAL_MAX_DELAY); // receive the status register value
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	// Wait until the busy bit is cleared
	while (status & W25X40_WIP_MASK)
	{
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
		HAL_SPI_Transmit(hspi, &temp, 1, HAL_MAX_DELAY); // send read status register command
		HAL_SPI_Receive(hspi, &status, 1, HAL_MAX_DELAY); // receive the status register value
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40
	}
}



/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_ReadData
 *Description: Reads data from specified address into data variable
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_ReadData(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint32_t size)
{
	uint8_t command[4];

	// Send read data command
	command[0] = 0x03;
	command[1] = (address & 0xFF0000) >> 16;
	command[2] = (address & 0xFF00) >> 8;
	command[3] = (address & 0xFF);

	//Send command->address
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(hspi, command, 4, HAL_MAX_DELAY);
	// Receive the data
	HAL_SPI_Receive(hspi, data, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40


}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_PageProgram
 *Description: Writes data to specified address, needing to specify the data and size of the data.
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_PageProgram(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint32_t size)
{
	uint8_t command[3];

	// Send write enable command
	W25X40_WriteEnable(hspi);

	// Send page program command
	command[0] = 0x02;
	command[1] = (address & 0xFF0000) >> 16;
	command[2] = (address & 0xFF00) >> 8;
	command[3] = (address & 0xFF);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(hspi, command, sizeof(command) + 1, HAL_MAX_DELAY);

	// Send the data
	HAL_SPI_Transmit(hspi, data, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	// Wait for the program operation to complete
	W25X40_Busy_Check(hspi);


}


/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_PageProgram
 *Description: Writes data to specified address, needing to specify the data and size of the data.
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_PageProgram2(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint32_t size)
{
	uint8_t command[3];


	// Send page program command
	command[0] = 0x02;
	command[1] = (address & 0xFF0000) >> 16;
	command[2] = (address & 0xFF00) >> 8;
	command[3] = (address & 0xFF);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(hspi, command, sizeof(command) + 1, HAL_MAX_DELAY);

	// Send the data
	HAL_SPI_Transmit(hspi, data, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	// Wait for the program operation to complete
	W25X40_Busy_Check(hspi);


}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Set_Status
 *Description: Sets the status register of the W25X40 to be able to page program all memory locations
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Set_Status(SPI_HandleTypeDef *spi_instance)
{
	uint8_t cmd=0b10000000;//unlocks all memory locations
	uint8_t temp = 0x01;//temp variable to write to status reg

	W25X40_WriteEnable(spi_instance);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(spi_instance, &temp, 1, HAL_MAX_DELAY); // send read status register command
	HAL_SPI_Transmit(spi_instance, &cmd, 1, HAL_MAX_DELAY); // send read status register command
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	W25X40_Busy_Check(spi_instance);
}



/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Chip_Erase
 *Description: Sets the status register of the W25X40 to be able to page program all memory locations
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Chip_Erase(SPI_HandleTypeDef *spi_instance)
{

	W25X40_WriteEnable(spi_instance);
	uint8_t cmd=0x60;//unlocks all memory locations


	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(spi_instance, &cmd, 1, HAL_MAX_DELAY); // send read status register command
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	W25X40_Busy_Check(spi_instance);
	num_entries=0;
}


/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Chip_Erase
 *Description: Sets the status register of the W25X40 to be able to page program all memory locations
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Multi_Write_Data(SPI_HandleTypeDef *spi_instance, uint32_t address, uint8_t *data, uint32_t len)
{
	uint8_t cmd[4];
	uint32_t page_offset, page_remain, sector_offset, sector_remain, bytes_to_write;




	// Enable chip select for the memory chip
	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	// Send "page program" command
	cmd[0] = 0x02;

	while (len > 0) {

		// Send "write enable" command
		W25X40_WriteEnable(spi_instance);
		// Calculate the page offset and number of bytes remaining in the current page
		page_offset = address % W25X40_PAGE_SIZE;
		page_remain = W25X40_PAGE_SIZE - page_offset;

		// Calculate the sector offset and number of bytes remaining in the current sector
		sector_offset = address % W25X40_SECTOR_SIZE;
		sector_remain = W25X40_SECTOR_SIZE - sector_offset;

		// Calculate the number of bytes to write in the current iteration
		bytes_to_write = (len < page_remain) ? len : page_remain;
		bytes_to_write = (bytes_to_write < sector_remain) ? bytes_to_write : sector_remain;

		// Send address bytes
		cmd[1] = (address >> 16) & 0xFF;
		cmd[2] = (address >> 8) & 0xFF;
		cmd[3] = address & 0xFF;
		W25X40_PageProgram2(spi_instance,address,data,bytes_to_write);

		// Wait for write operation to complete
		//while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {}

		// Update variables
		address += bytes_to_write;
		data += bytes_to_write;
		len -= bytes_to_write;
	}

	// Disable chip select for the memory chip
	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}



/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Directory_Read
 *Description: Reads and displays directory information, printing to terminal at 115200 BR. Called at startup to discern
 *	 	 	   which memory locations are available for new storage
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Directory_Read(SPI_HandleTypeDef *hspi)
{

	//variable to use for transmitting
	uint8_t dev_cmd[4];
	uint8_t dev_res[4];
	uint8_t i =0;
	uint32_t temp = 0;

	printf("\n\r------------------------------------------------------------\n\r");
	/////////////////////////////////////////Read Available Data///////////////////////////////////////////

	W25X40_Available_MEM(hspi);

	/////////////////////////////////////////Print Number of Entries////////////////////////////////////////

	printf("Number of Directory Entries: %x\n\r",num_entries);

	////////////////////////////////////Print Entries an Associated Addresses//////////////////////////////
	if(num_entries == 0)
	{
		printf("No Existing Entries to Display\n\r");
	}

	else{
		uint32_t address = entry1;
		for(i=0;i<10;i++)
		{
			dev_cmd[0] = 0x03;
			dev_cmd[1] = (address & 0xFF0000) >> 16;
			dev_cmd[2] = (address & 0xFF00) >> 8;
			dev_cmd[3] = (address & 0xFF);

			//Send command->address->wait for busy flag to reset
			HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
			HAL_SPI_Transmit(hspi, dev_cmd, 4, HAL_MAX_DELAY);
			// Receive the data
			HAL_SPI_Receive(hspi, dev_res, 3, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40
			W25X40_Busy_Check(hspi);
			temp =((uint32_t)dev_res[0] << 16) | ((uint32_t)dev_res[1] << 8) | dev_res[2];
			if(temp == 0xffffff)
			{
				printf("%d: Empty\n\r",i+1);
			}
			else
			{
				printf("%d: ",i+1);
				W25X40_Print_Title(hspi, address);
			}
			address = address + 0x1000;
		}
		printf("Number of available memory locations: %X\n\r",(10-num_entries));
		printf("\n\r------------------------------------------------------------\n\r");



	}
}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Chip_Erase
 *Description: Sets the status register of the W25X40 to be able to page program all memory locations
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Entry_Delete(SPI_HandleTypeDef *spi_instance, uint8_t num_Entry)
{
	uint32_t dir_address;
	uint32_t address_to_erase;
	uint8_t dev_cmd[4];
	uint8_t dev_res[4];
	uint8_t temp[1];
	uint32_t temp2 = 0;

	if (num_Entry <= 10 && num_Entry >= 1) {
		address_to_erase = (num_Entry * 0x001000) + 0x010000;
		dir_address = (num_Entry * 0x001000) + 0x001000;
	}
	else {
		return;
	}

	dev_cmd[0] = 0x03;
	dev_cmd[1] = (address_to_erase & 0xFF0000) >> 16;
	dev_cmd[2] = (address_to_erase & 0xFF00) >> 8;
	dev_cmd[3] = (address_to_erase & 0xFF);

	//Send command->address->wait for busy flag to reset
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(spi_instance, dev_cmd, 4, HAL_MAX_DELAY);
	// Receive the data
	HAL_SPI_Receive(spi_instance, dev_res, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40
	W25X40_Busy_Check(spi_instance);
	temp2 =((uint32_t)dev_res[0] << 16) | ((uint32_t)dev_res[1] << 8) | dev_res[2];
	if(temp2 == 0xffffff)
	{
		printf("No data to delete\n\r");
	}

	else{
	//erase sector of memory associated with directory entry and entry itself
	W25X40_SectorErase(spi_instance,address_to_erase);
	W25X40_SectorErase(spi_instance,dir_address);

	//clear the entry from the directory and update variables regarding memory size and number of total entries
	num_entries = num_entries-1; //Decrement number of entries
	temp[0]= num_entries;

	W25X40_Busy_Check(spi_instance);
	}
}



/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Available_MEM
 *Description: Reads the start of every 4kb sector checking if the beginning is cleared. Then keeping tally to know how much is available
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Available_MEM(SPI_HandleTypeDef *hspi)
{
	uint8_t command[4];
	uint32_t i =0;
	uint8_t temp = 0;
	data_storage = 0;

	for(i=0;i<0x07FFFF;)
	{
		// Send read data command
		command[0] = 0x03;
		command[1] = (i & 0xFF0000) >> 16;
		command[2] = (i & 0xFF00) >> 8;
		command[3] = (i & 0xFF);

		//Send command->address
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
		HAL_SPI_Transmit(hspi, command, 4, HAL_MAX_DELAY);
		// Receive the data
		HAL_SPI_Receive(hspi, &temp, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

		i = i+0x1000;//increment counter by 4kbyte amount to check next sector


		if(temp == 0xFF)//if first entry in sector is 0xFF it is assumed to be clear
		{
			data_storage = data_storage + 4000;//increment amount of available data if first entry erased
		}
	}

	printf("Available Memory (bytes):%ld\n\r",data_storage);
	printf("Memory Used (bytes):%ld\n\r",512000-data_storage);

	command[0] = (data_storage & 0xFF0000) >> 16;
	command[1] = (data_storage & 0xFF00) >> 8;
	command[2] = (data_storage & 0xFF);

	W25X40_PageProgram(hspi, mem_used_addr, command, 3);

}


void W25X40_WriteString(char *str)
{
	uint8_t i = 0;
	uint16_t strLength = strlen(str);

	// Convert the string to a byte array
	for (i = 0; i < strLength; i++) {
		string_to_array[i] = (uint8_t)str[i];
		string_length = i;
	}
	string_to_array[i + 1] = '\0';

}

void W25X40_Print_Title(SPI_HandleTypeDef *hspi, uint32_t address)
{

	uint8_t command[4];
	uint8_t print_array[256]={0};
	uint8_t rxData[256];
	uint8_t i = 0;

	// Send read data command



	command[0] = 0x03;
	command[1] = (address & 0xFF0000) >> 16;
	command[2] = (address & 0xFF00) >> 8;
	command[3] = (address & 0xFF);
	//Send command->address
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
	HAL_SPI_Transmit(hspi, command, 4, HAL_MAX_DELAY);
	// Receive the data
	HAL_SPI_Receive(hspi, rxData, 256, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

	while(rxData[i] != '\n')
	{
		print_array[i]=rxData[i];
		i++;
	}

	print_array[i+1]='\0';

	printf("%s\n\r",print_array);

}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Available_Entries
 *Description: Reads the directory entry locations to see if entry is there. Call at the beginning of main to set global var
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Available_Entries(SPI_HandleTypeDef *hspi)
{
	uint8_t command[4];
	uint32_t i =0;
	uint32_t address= entry1;
	uint8_t temp = 0;
	data_storage = 0;

	for(i=0;i<10;i++)
	{
		// Send read data command
		command[0] = 0x03;
		command[1] = (address & 0xFF0000) >> 16;
		command[2] = (address & 0xFF00) >> 8;
		command[3] = (address & 0xFF);

		//Send command->address
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
		HAL_SPI_Transmit(hspi, command, 4, HAL_MAX_DELAY);
		// Receive the data
		HAL_SPI_Receive(hspi, &temp, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

		address = address+0x1000;//increment counter by 4kbyte amount to check next sector


		if(temp != 0xFF)//if first entry in sector is 0xFF it is assumed to be clear
		{
			num_entries = num_entries + 1;//increment amount of available data if first entry erased
		}
	}

}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Store_Data_Entry
 *Description: Takes in 256 bytes x 15 array and program to first available memory address
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Store_Data(SPI_HandleTypeDef *hspi, uint8_t line_Num)//add non global data pass
{
	uint8_t dev_cmd[4];//variables for sending and recieving data from uart
	uint8_t dev_res[4];
	uint8_t data[256];
	uint8_t i = 0;
	uint32_t temp = 0;
	int row_index = 0; // index of the row to access

	uint32_t address = entry1;//set address to first used and begin to check for open area

	for(i=0;i<10;i++)
	{
		dev_cmd[0] = 0x03;
		dev_cmd[1] = (address & 0xFF0000) >> 16;
		dev_cmd[2] = (address & 0xFF00) >> 8;
		dev_cmd[3] = (address & 0xFF);

		//Send command->address->wait for busy flag to reset
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
		HAL_SPI_Transmit(hspi, dev_cmd, 4, HAL_MAX_DELAY);

		// Receive the data
		HAL_SPI_Receive(hspi, dev_res, 3, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40

		W25X40_Busy_Check(hspi);

		temp =((uint32_t)dev_res[0] << 16) | ((uint32_t)dev_res[1] << 8) | dev_res[2];//combine data into one number

		if(temp == 0xffffff)//find first available
		{
			num_entries++;
			break;
		}
		else if (i >= 9)
		{
			printf("\r\nERROR - No available memory to store. Please Delete an entry and retry.\n\r");
		}
		else {
			address += 0x1000;
		}
	}
	//get data into array to pass to page program
	for(row_index = 1; row_index < line_Num; row_index++)
	{
		for (int i = 0; i < 256; i++)
		{
			data[i] = fullFileBuffer[row_index][i];//CHANGE ARRAY VARIABLE!!!!!!!
		}

		W25X40_PageProgram(hspi, address, data, 256);
		address += 256; //increment by 256 to send next line of data
	}
}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *Function:W25X40_Read_Data_Entry
 *Description: Takes in entry number and spi handle to read directory entry
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void W25X40_Read_Data_Entry(SPI_HandleTypeDef *hspi,uint8_t num_Entry)//add non global data pass
{

	uint8_t dev_cmd[4];
	uint8_t dev_res[4];
	uint8_t i =0;
	uint32_t temp = 0;

	uint32_t address;


	if(num_Entry==1){address = entry1;}
	if(num_Entry==2){address = entry2;}
	if(num_Entry==3){address = entry3;}
	if(num_Entry==4){address = entry4;}
	if(num_Entry==5){address = entry5;}
	if(num_Entry==6){address = entry6;}
	if(num_Entry==7){address = entry7;}
	if(num_Entry==8){address = entry8;}
	if(num_Entry==9){address = entry9;}
	if(num_Entry==10){address = entry10;}


	for(i=0;i<15;i++)
	{
		//check if data to read
		dev_cmd[0] = 0x03;
		dev_cmd[1] = (address & 0xFF0000) >> 16;
		dev_cmd[2] = (address & 0xFF00) >> 8;
		dev_cmd[3] = (address & 0xFF);

		//Send command->address->wait for busy flag to reset
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the W25X40
		HAL_SPI_Transmit(hspi, dev_cmd, 4, HAL_MAX_DELAY);

		// Receive the data
		HAL_SPI_Receive(hspi, dev_res, 3, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the W25X40
		W25X40_Busy_Check(hspi);

		temp =((uint32_t)dev_res[0] << 16) | ((uint32_t)dev_res[1] << 8) | dev_res[2];

		if(temp == 0xffffff)
		{
			break;//no more stuff to read
		}
		else
		{
			W25X40_Print_Title(hspi, address);//function that will read from address up till newline character
			address = address + 256;//increment to next line
		}
	}

}


