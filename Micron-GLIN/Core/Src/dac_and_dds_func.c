#include "dac_and_dds_func.h"

extern float DDS_clock_frequecny;
extern float DAC_fullrange_voltage;

extern uint32_t DAC_code;
extern uint8_t CPLD_WORD;

float corr_coeff_1;
float corr_coeff_2;
float corr_coeff_3;

extern uint8_t eta_hours,eta_minute,eta_second;
extern FunctionalState DAC_code_direction;

extern float DDS_target_frequecny;
extern float DAC_target_speed;
extern uint32_t DDS_target_multipiller;

extern uint32_t DAC_tx_buffer;
extern uint16_t DAC_tx_tmp_buffer[2];
extern float DDS_FTW;


//==============================================================================================
void Relay_control(uint8_t relay,uint8_t state){
	int Relay_address=0;
	if(relay<1 || relay>3 || state>1) return;

	while(Relay_address<=0x5) // Set all OUTx to zero
	{
		HAL_GPIO_WritePin(Control_bus_1_GPIO_Port, Control_bus_1_Pin,  Relay_address & 0x1     );
		HAL_GPIO_WritePin(Control_bus_2_GPIO_Port, Control_bus_2_Pin, (Relay_address & 0x2) >>1);
		HAL_GPIO_WritePin(Control_bus_3_GPIO_Port, Control_bus_3_Pin, (Relay_address & 0x4) >>2);

		HAL_GPIO_WritePin(Control_bus_0_GPIO_Port, Control_bus_0_Pin, 0); // LVL 0

		HAL_Delay(1); // wait 1ms
		HAL_GPIO_WritePin(Relay_cs_GPIO_Port, Relay_cs_Pin, 0); // Send strobe
		HAL_Delay(1); // wait 1ms
		HAL_GPIO_WritePin(Relay_cs_GPIO_Port, Relay_cs_Pin, 1);
		HAL_Delay(1); // wait 1ms
		Relay_address++;
	}


	switch (relay)
	{
	case 3:
		if (state==1){
			Relay_address=0x5; //OUT6
		}else{
			Relay_address=0x4; //OUT5
		} break;
	case 2:
		if (state==1){
			Relay_address=0x3; //OUT4
		}else{
			Relay_address=0x2; //OUT3
		} break;
	case 1:
		if (state==1){
			Relay_address=0x1; //OUT2
		}else{
			Relay_address=0x0; //OUT1
		} break;
	}

	HAL_GPIO_WritePin(Control_bus_1_GPIO_Port, Control_bus_1_Pin,  Relay_address & 0x1     );
	HAL_GPIO_WritePin(Control_bus_2_GPIO_Port, Control_bus_2_Pin, (Relay_address & 0x2) >>1);
	HAL_GPIO_WritePin(Control_bus_3_GPIO_Port, Control_bus_3_Pin, (Relay_address & 0x4) >>2);

	HAL_GPIO_WritePin(Control_bus_0_GPIO_Port, Control_bus_0_Pin, 1); // LVL 1

	HAL_Delay(1); // wait 1ms
	HAL_GPIO_WritePin(Relay_cs_GPIO_Port, Relay_cs_Pin, 0); // Send strobe
	HAL_Delay(1); // wait 1ms
	HAL_GPIO_WritePin(Relay_cs_GPIO_Port, Relay_cs_Pin, 1); // End strobe
	HAL_Delay(50); // wait 50ms

	HAL_GPIO_WritePin(Control_bus_0_GPIO_Port, Control_bus_0_Pin, 0); // LVL 0

	HAL_Delay(1); // wait 1ms
	HAL_GPIO_WritePin(Relay_cs_GPIO_Port, Relay_cs_Pin, 0); // Send strobe
	HAL_Delay(1); // wait 1ms
	HAL_GPIO_WritePin(Relay_cs_GPIO_Port, Relay_cs_Pin, 1); // End strobe
	HAL_Delay(1); // wait 1ms

}

//==============================================================================================



//==============================================================================================
void CPLD_control(uint8_t divide_coeff){
	if(divide_coeff>0x0F) return;

	HAL_GPIO_WritePin(Control_bus_0_GPIO_Port, Control_bus_0_Pin,  divide_coeff & 0x1     );
	HAL_GPIO_WritePin(Control_bus_1_GPIO_Port, Control_bus_1_Pin, (divide_coeff & 0x2) >>1);
	HAL_GPIO_WritePin(Control_bus_2_GPIO_Port, Control_bus_2_Pin, (divide_coeff & 0x4) >>2);
	HAL_GPIO_WritePin(Control_bus_3_GPIO_Port, Control_bus_3_Pin, (divide_coeff & 0x8) >>3);
	HAL_GPIO_WritePin(Count_EN_GPIO_Port, Count_EN_Pin, GPIO_PIN_SET); // Send strobe
	HAL_GPIO_WritePin(Count_EN_GPIO_Port, Count_EN_Pin, GPIO_PIN_RESET);

	if(divide_coeff==0x00)
	{
		cfg.LDACMODE=0;
	}
	else
	{
		cfg.LDACMODE=1;
	}
}

//==============================================================================================



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
	DAC_code=value;
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
	DAC_tx_buffer+=(cfg.VREFVAL & 0x06)<<6;
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

	uint16_t spi_receive[2]={0x0,0x0},DAC_tx_tmp_buffer2[2],ALM=0;

	//CPLD_control(0x0); // Disable LDAC signal

	cfg.EN_TMP_CAL=1;
	DAC_SendInit();

	DAC_tx_buffer=0x04000100; // Write TRIGGER RCLTMP

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);

	DAC_tx_buffer=0x85000000; // read status register

	DAC_tx_tmp_buffer2[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
	DAC_tx_tmp_buffer2[1]=(DAC_tx_buffer & 0x0000FFFF);

	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,2,2);
	HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

	do{ // Check complete flag
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer2,2,2);
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1,(uint8_t *)spi_receive, 2, 2);
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
		ALM=(spi_receive[1] & 0x1000) >> 12;
		if(ALM!=1)HAL_Delay(10);
	}while(ALM!=1);
}

void DDS_Calculation(void)
{
	float hw_limit=1000; // 1(256)kHz hardware optimized limit
	float dac_counts=1048576;
	float corr_coeff;
	float dac_tmp=DAC_code;
	float second_left;
	uint32_t codes_left;

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

	DDS_FTW=(((DDS_target_frequecny/corr_coeff)*((1<<CPLD_WORD)+1))/DDS_clock_frequecny)*(float)0xFFFFFFFF;

	if(DAC_code_direction)
	{
		codes_left=0xFFFFF-DAC_code;
	}
	else
	{
		codes_left=DAC_code;
	}

	second_left=codes_left/DDS_target_multipiller/DDS_target_frequecny;
	eta_second=(uint32_t)second_left % 60;
	eta_minute=(uint32_t)(second_left / 60) % 60;
	eta_hours=(uint32_t) second_left / 3600;
}

//==============================================================================================
void DDS_Init(void)
{
	uint16_t DDS_tx_buffer[1];
	DDS_Calculation();

	HAL_Delay(100);

	//CONTROL REGISTER WRITE SLEEP =1 ,	RESET = 1,	CLR = 1
	DDS_tx_buffer[0]=0xC000; // Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]+=0x7 << 11; //  SLEEP = 1 , RESET = 1,	CLR = 1
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);

	//DO NOT !!! SET SYNC AND/OR SELSRC TO 1

	//WRITE INITIAL DATA

	// Write to Frequency 0 Reg, H MSB
	DDS_tx_buffer[0]=0x3300;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 24) & 0xFF;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	// Write to Frequency 0 Reg, L MSBs
	DDS_tx_buffer[0]=0x2200;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	// Write to Frequency 0 Reg, H LSBs
	DDS_tx_buffer[0]=0x3100;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	// Write to Frequency 0 Reg, L LSBs
	DDS_tx_buffer[0]=0x2000;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW & 0xFF);

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);


	HAL_Delay(100);

	// CONTROL REGISTER WRITE, 	SLEEP = 0,	RESET = 0, CLR = 0

	// Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]=0xC000; // Exit DAC from Sleep+Reset mode

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);

}

//==============================================================================================
void DDS_Update(void)
{
	uint16_t DDS_tx_buffer[1];

	DDS_Calculation();

	// Write to Frequency 0 Reg, H MSB
	DDS_tx_buffer[0]=0x3300;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 24) & 0xFF;
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

	// Write to Frequency 0 Reg, L MSBs
	DDS_tx_buffer[0]=0x2200;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

	// Write to Frequency 0 Reg, H LSBs
	DDS_tx_buffer[0]=0x3100;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

	// Write to Frequency 0 Reg, L LSBs
	DDS_tx_buffer[0]=0x2000;
	DDS_tx_buffer[0]+=((uint32_t)DDS_FTW & 0xFF);

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

/*	// Control DDS (D15=1, D14=1)
	DDS_tx_buffer[0]=0x9000; // Latch to output by synchonizing data. In this case, the SELSRC bit is again set to 1 using Command Bits [1:0] for C15 and C14.

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,1,5);
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
*/
}


