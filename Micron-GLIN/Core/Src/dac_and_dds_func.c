#include "dac_and_dds_func.h"

//==============================================================================================
void DAC_Write(uint32_t value)
{

	DAC_tx_buffer=0x01000000; // Write DAC-DATA
	DAC_tx_buffer+=(value & 0xFFFFF)<<4;

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);

	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,2,2);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
}

//==============================================================================================
void DAC_Write_FAST(void)
{
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,2,2);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
}

//==============================================================================================
void DAC_SendInit(void)
{

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

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);


	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,2,5);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

}

//==============================================================================================
void DAC_TEMP_CAL(void)
{
	uint32_t DAC_tx_buffer;
	uint16_t DAC_tx_tmp_buffer[2];

	uint8_t OK[]="OK\n\r";
	uint8_t run_cal[]="\r\nCalibration in progress..";

	uint16_t spi_receive[2]={0x0,0x0},DAC_tx_tmp_buffer2[2],ALM=0;

	uint8_t count_tmp=HAL_GPIO_ReadPin(COUNT_EN_GPIO_Port, COUNT_EN_Pin); // Save LDAC signal state

//	DDS_prepare_to_tempcal();

	HAL_GPIO_WritePin(COUNT_EN_GPIO_Port, COUNT_EN_Pin, GPIO_PIN_SET); // Disable LDAC signal

	cfg.EN_TMP_CAL=1;
	DAC_SendInit();
	HAL_Delay(10);

	DAC_tx_buffer=0x04000100; // Write TRIGGER RCLTMP

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);

	DAC_tx_buffer=0x85000000; // read status register

	DAC_tx_tmp_buffer2[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
	DAC_tx_tmp_buffer2[1]=(DAC_tx_buffer & 0x0000FFFF);

	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,2,2);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

	HAL_Delay(10);
	CDC_Transmit_FS(run_cal, strlen((const char *)run_cal));
	HAL_Delay(500); // Wait some time....

	do{ // Check complete flag
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer2,2,2);
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1,(uint8_t *)spi_receive, 2, 2);
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
		ALM=(spi_receive[1] & 0x1000) >> 12;
		if(ALM!=1)HAL_Delay(1000);
	}while(ALM!=1);

	HAL_Delay(10);
	CDC_Transmit_FS(OK, strlen((const char *)OK));
	HAL_Delay(10);

	DDS_Init();
	HAL_GPIO_WritePin(COUNT_EN_GPIO_Port, COUNT_EN_Pin, count_tmp); // Back LDAC signal state
}

//==============================================================================================
void DDS_Init(void)
{
	uint16_t DDS_tx_buffer[1];
	float hw_limit=1000; // 1(256)kHz hardware optimized limit
	float dac_counts=1048576;


	DDS_target_frequecny=dac_counts/(DAC_fullrange_voltage/DAC_target_speed);

	if(DDS_target_frequecny>hw_limit)
	{
		DDS_target_multipiller=DDS_target_frequecny/hw_limit;
		DDS_target_frequecny=dac_counts/(DAC_fullrange_voltage/DAC_target_speed);
		DDS_target_frequecny/=(float)DDS_target_multipiller;
	} else DDS_target_multipiller = 1;

	float DDS_FTW=((DDS_target_frequecny*256)/DDS_clock_frequecny)*(float)0xFFFFFFFF;

	// Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]=0xC000; // Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]+=1 << 12; // Enter DDS to Reset mode, RST (D12) = 1

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, H MSB
	DDS_tx_buffer[0]=0x3300;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 24) & 0xFF;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, L MSBs
	DDS_tx_buffer[0]=0x2200;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, H LSBs
	DDS_tx_buffer[0]=0x3100;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, L LSBs
	DDS_tx_buffer[0]=0x2000;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW & 0xFF);

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]=0xC000; // Exit DAC from Sleep+Reset mode

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

}

//==============================================================================================
void DDS_Update(void)
{
	uint16_t DDS_tx_buffer[1];
	float hw_limit=1000; // 1(256)kHz hardware optimized limit
	float dac_counts=1048576;
	float corr_coeff;
	float dac_tmp=DAC_code;

	corr_coeff=corr_coeff_1*dac_tmp*dac_tmp;
	corr_coeff+=corr_coeff_2*dac_tmp;
	corr_coeff+=corr_coeff_3;

	DDS_target_frequecny=dac_counts/(DAC_fullrange_voltage/DAC_target_speed);

	if(DDS_target_frequecny>hw_limit)
	{
		DDS_target_multipiller=DDS_target_frequecny/hw_limit;
		DDS_target_frequecny=dac_counts/(DAC_fullrange_voltage/DAC_target_speed);
		DDS_target_frequecny/=(float)DDS_target_multipiller;
	} else DDS_target_multipiller = 1;

	float DDS_FTW=(((DDS_target_frequecny/corr_coeff)*256)/DDS_clock_frequecny)*(float)0xFFFFFFFF;

	// Write to Frequency 0 Reg, H MSB
	DDS_tx_buffer[0]=0x3300;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 24) & 0xFF;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, L MSBs
	DDS_tx_buffer[0]=0x2200;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, H LSBs
	DDS_tx_buffer[0]=0x3100;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Write to Frequency 0 Reg, L LSBs
	DDS_tx_buffer[0]=0x2000;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW & 0xFF);

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

	// Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]=0x9000; // Latch to output by synchonizing data. In this case, the SELSRC bit is again set to 1 using Command Bits [1:0] for C15 and C14.

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

}


/*
//==============================================================================================
void DDS_prepare_to_tempcal(void)
{
	uint16_t DDS_tx_buffer[6];

	DDS_target_frequecny=0.1;

	float DDS_FTW=((DDS_target_frequecny*256)/DDS_clock_frequecny)*(float)0xFFFFFFFF;

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
*/
