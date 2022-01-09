#include "dac_and_dds_func.h"

//==============================================================================================
void DAC_Write(uint32_t value)
{
	uint32_t DAC_tx_buffer;
	uint8_t DAC_tx_tmp_buffer[4];

	// Speedup hint: Calculate data AFTER send.
	DAC_tx_buffer=0x01000000; // Write DAC-DATA
	DAC_tx_buffer+=(value & 0xFFFFF)<<4;

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer[3]=(DAC_tx_buffer & 0x000000FF);

	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,4,2);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
}


//==============================================================================================
void DAC_SendInit(void)
{
	uint32_t DAC_tx_buffer;
	uint8_t DAC_tx_tmp_buffer[4];

	DAC_tx_buffer=0x02000000; // Write CONFIG1
	DAC_tx_buffer+=(cfg.PDN & 0x01)<<4;
	DAC_tx_buffer+=(cfg.VREFVAL & 0x0F)<<6;
	DAC_tx_buffer+=(cfg.FSET & 0x01)<<10;
	DAC_tx_buffer+=(cfg.DSDO & 0x01)<<11;
	DAC_tx_buffer+=(cfg.ENALMP & 0x01)<<12;
	DAC_tx_buffer+=(cfg.FSDO & 0x01)<<13;
	DAC_tx_buffer+=(cfg.LDACMODE & 0x01)<<14;
	DAC_tx_buffer+=(cfg.TNH_MASK & 0x03)<<18;
	DAC_tx_buffer+=(cfg.EN_TMP_CAL & 0x01)<<23;

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer[3]=(DAC_tx_buffer & 0x000000FF);


	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,4,5);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

}

//==============================================================================================
void DAC_TEMP_CAL(void)
{
	uint32_t DAC_tx_buffer;
	uint8_t DAC_tx_tmp_buffer[4];

	uint8_t OK[]="\r\n OK \n\rEnter command: ";
	uint8_t run_cal[]="\r\nCalibration in progress...";

	uint8_t spi_receive[4],DAC_tx_tmp_buffer2[4],ALM=0;

	uint8_t count_tmp=HAL_GPIO_ReadPin(COUNT_EN_GPIO_Port, COUNT_EN_Pin); // Save LDAC signal state

	HAL_GPIO_WritePin(COUNT_EN_GPIO_Port, COUNT_EN_Pin, GPIO_PIN_SET); // Disable LDAC signal

	HAL_Delay(10);
	CDC_Transmit_FS(run_cal, strlen((const char *)run_cal));
	HAL_Delay(10);

	cfg.EN_TMP_CAL=1;
	DAC_SendInit();

	for(int i=0;i<4;i++)spi_receive[i]=0;

	DAC_tx_buffer=0x04000100; // Write TRIGGER RCLTMP

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer[3]=(DAC_tx_buffer & 0x000000FF);

	DAC_tx_buffer=0x85000000; // read status register

	DAC_tx_tmp_buffer2[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer2[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer2[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer2[3]=(DAC_tx_buffer & 0x000000FF);

	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,4,2);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

	do{

		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer2,4,2);
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);


		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1,(uint8_t *)spi_receive, 4, 2);
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
		ALM=(spi_receive[2] & 0x10) >> 4;
		if(ALM!=1)HAL_Delay(1000);
	}while(ALM!=1);

	HAL_Delay(10);
	CDC_Transmit_FS(OK, strlen((const char *)OK));
	HAL_Delay(10);

	HAL_GPIO_WritePin(COUNT_EN_GPIO_Port, COUNT_EN_Pin, count_tmp); // Back LDAC signal state
}

//==============================================================================================
void DDS_Init(void)
{
	uint16_t DDS_tx_buffer[6];

	DDS_target_frequecny=0xFFFFF/(DAC_fullrange_voltage/DAC_target_speed);

	if((DDS_target_frequecny*256)>500000)
	{
		DDS_target_multipiller=(DDS_target_frequecny*256)/500000;
		DDS_target_frequecny=0xFFFFF/(DAC_fullrange_voltage/DAC_target_speed)/DDS_target_multipiller;
	} else DDS_target_multipiller = 1;

	float DDS_FTW=((DDS_target_frequecny*256)/DDS_clock_frequecny)*0xFFFFFFFF;

	DDS_tx_buffer[0]=0xF800; // Enter DAC to Sleep+Reset mode

	DDS_tx_buffer[1]=0x3300; // Write to Frequency 0 Reg, H MSB
	DDS_tx_buffer[1]+=((uint32_t)DDS_FTW >> 24) & 0xFF;

	DDS_tx_buffer[2]=0x2200; // Write to Frequency 0 Reg, L MSBs
	DDS_tx_buffer[2]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

	DDS_tx_buffer[3]=0x3100; // Write to Frequency 0 Reg, H LSBs
	DDS_tx_buffer[3]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

	DDS_tx_buffer[4]=0x2000; // Write to Frequency 0 Reg, L LSBs
	DDS_tx_buffer[4]+=((uint32_t)DDS_FTW & 0xFF);

	DDS_tx_buffer[5]=0xC000; // Exit DAC from Sleep+Reset mode

	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,6,100);

}

//==============================================================================================
void DDS_prepare_to_tempcal(void)
{
	uint16_t DDS_tx_buffer[6];

	DDS_target_frequecny=0.01;

	float DDS_FTW=((DDS_target_frequecny*256)/DDS_clock_frequecny)*0xFFFFFFFF;

	DDS_tx_buffer[0]=0xF800; // Enter DAC to Sleep+Reset mode

	DDS_tx_buffer[1]=0x3300; // Write to Frequency 0 Reg, H MSB
	DDS_tx_buffer[1]+=((uint32_t)DDS_FTW >> 24) & 0xFF;

	DDS_tx_buffer[2]=0x2200; // Write to Frequency 0 Reg, L MSBs
	DDS_tx_buffer[2]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

	DDS_tx_buffer[3]=0x3100; // Write to Frequency 0 Reg, H LSBs
	DDS_tx_buffer[3]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

	DDS_tx_buffer[4]=0x2000; // Write to Frequency 0 Reg, L LSBs
	DDS_tx_buffer[4]+=((uint32_t)DDS_FTW & 0xFF);

	DDS_tx_buffer[5]=0xC000; // Exit DAC from Sleep+Reset mode

	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,6,100);
	HAL_GPIO_WritePin(COUNT_EN_GPIO_Port, COUNT_EN_Pin, GPIO_PIN_RESET); // Enable LDAC signal

	while(HAL_GPIO_ReadPin(CPU_LDAC_GPIO_Port, CPU_LDAC_Pin)==GPIO_PIN_RESET); // Waiting LDAC become high

	DDS_tx_buffer[0]=0xF800; // Enter DAC to Sleep+Reset mode
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,100);

}
