#include "MPU6050.h"
#include "main.h"


extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

void MX_I2C1_Init(void);

Struct_MPU6050 MPU6050;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}



void MPU6050_Initialization(void)
{
    HAL_Delay(50);
    uint8_t who_am_i = 0;
    char message[100];

    // Send initial check message via UART
    /*snprintf(message, sizeof(message), "Checking MPU6050...\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

    // Read WHO_AM_I register to check MPU6050 identity
    MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);
    if(who_am_i == 0x68)
    {
        snprintf(message, sizeof(message), "MPU6050 who_am_i = 0x%02x...OK\n", who_am_i);
        HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    }
    else
    {
        snprintf(message, sizeof(message), "ERROR!\nMPU6050 who_am_i : 0x%02x should be 0x68\n", who_am_i);
        HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

        while(1)
        {
            snprintf(message, sizeof(message), "who am i error. Can not recognize MPU6050\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
            HAL_Delay(100);
        }
    }*/

    // Reset the whole module before initialization
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x1<<7);
    HAL_Delay(100);

    // Power Management setting: Wake up MPU6050 from sleep mode
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(50);

    // Set sample rate divider (200Hz for both accelerometer and gyroscope)
    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39);
    HAL_Delay(50);

    // Set DLPF (Digital Low Pass Filter) to 0 (no filtering)
    MPU6050_Writebyte(MPU6050_CONFIG, 0x00);
    HAL_Delay(50);

    // Gyroscope full scale range setting
    uint8_t FS_SCALE_GYRO = 0x0;  // +-250 degree/s
    MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO<<3);
    HAL_Delay(50);

    // Accelerometer full scale range setting
    uint8_t FS_SCALE_ACC = 0x2;   // +-8g
    MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC<<3);
    HAL_Delay(50);

    // Calculate LSB sensitivity for gyro and accelerometer
    MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
    // snprintf(message, sizeof(message), "LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n", LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC);
    // HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

    // Optionally, if you want to configure interrupts (commented out here)
    // uint8_t INT_LEVEL = 0x0;  // active high
    // uint8_t LATCH_INT_EN = 0x0;  // interrupt 50us pulse
    // uint8_t INT_RD_CLEAR = 0x1;  // interrupt cleared by read operation
    // MPU6050_Writebyte(MPU6050_INT_PIN_CFG, (INT_LEVEL<<7) | (LATCH_INT_EN<<5) | (INT_RD_CLEAR<<4));
    // HAL_Delay(50);

    // Enable interrupt (example)
    // uint8_t DATA_RDY_EN = 0x1; // enable
    // MPU6050_Writebyte(MPU6050_INT_ENABLE, DATA_RDY_EN);
    // HAL_Delay(50);

    snprintf(message, sizeof(message), "MPU6050 setting is finished\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}


/*Get Raw Data from sensor*/
void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050)
{
	uint8_t data[14];
	MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);

	mpu6050->acc_x_raw = (data[0] << 8) | data[1];
	mpu6050->acc_y_raw = (data[2] << 8) | data[3];
	mpu6050->acc_z_raw = (data[4] << 8) | data[5];

	mpu6050->temperature_raw = (data[6] << 8) | data[7];

	mpu6050->gyro_x_raw = ((data[8] << 8) | data[9]);
	mpu6050->gyro_y_raw = ((data[10] << 8) | data[11]);
	mpu6050->gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch(FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

/*Convert Unit. acc_raw -> g, gyro_raw -> degree per second*/
void MPU6050_DataConvert(Struct_MPU6050* mpu6050)
{
	//printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO,LSB_Sensitivity_ACC);
	mpu6050->acc_x = mpu6050->acc_x_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_y = mpu6050->acc_y_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_z = mpu6050->acc_z_raw / LSB_Sensitivity_ACC;

	mpu6050->temperature = (float)(mpu6050->temperature_raw)/340+36.53;

	mpu6050->gyro_x = mpu6050->gyro_x_raw / LSB_Sensitivity_GYRO;
	mpu6050->gyro_y = mpu6050->gyro_y_raw / LSB_Sensitivity_GYRO;
	mpu6050->gyro_z = mpu6050->gyro_z_raw / LSB_Sensitivity_GYRO;
}


int MPU6050_DataReady(void)
{
	//old school way
	/*
	static uint8_t INT_STATE_FLAG = 0;
	static uint8_t DATA_RDY_INT_FLAG = 0;
	static uint8_t INT_PIN = 0;
	INT_PIN = LL_GPIO_IsInputPinSet(MPU6050_INT_PORT, MPU6050_INT_PIN);
	if(INT_PIN == 1)
	{
		MPU6050_Readbyte(MPU6050_INT_STATUS, &INT_STATE_FLAG); //flag cleared automatically within the sensor
		DATA_RDY_INT_FLAG = INT_STATE_FLAG & 0x01;
		if(DATA_RDY_INT_FLAG == 1)
		{
			INT_STATE_FLAG = 0; //flag clearing
			DATA_RDY_INT_FLAG = 0;
			INT_PIN = 0;
			return 1;
		}
	}
	return 0;
	 */
	return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}

void MPU6050_ProcessData(Struct_MPU6050* mpu6050)
{
	MPU6050_Get6AxisRawData(mpu6050);
	MPU6050_DataConvert(mpu6050);
}


