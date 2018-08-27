#include "bsp_imu.h"
#include "ist8310_reg.h"
#include <math.h>
#include "mpu6500_reg.h"
#include "spi.h"
#include "string.h"
#include "math.h"

static uint8_t tx, rx;
static uint8_t tx_buff[14];

uint8_t        mpu_buff[14];
mpu_data_t     mpu_data;
imu_data_t     imu;

uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data)
{
	GPIO_ResetBits(GPIOF , GPIO_Pin_6); 	
	tx = reg & 0x7F;
	rx = SPI5_ReadWriteByte(tx);
	tx = data;
	rx = SPI5_ReadWriteByte(tx);
	GPIO_SetBits(GPIOF , GPIO_Pin_6); 	
	return 0;
}

uint8_t mpu_read_reg(uint8_t const reg)
{
	GPIO_ResetBits(GPIOF , GPIO_Pin_6); 	
	tx = reg | 0x80;
	rx = SPI5_ReadWriteByte(tx);
	rx = SPI5_ReadWriteByte(tx);
	GPIO_SetBits(GPIOF , GPIO_Pin_6); 	
	return 0;
}

static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
  //turn off slave 1 at first
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  mpu_write_reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  mpu_write_reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  //turn on slave 1 with one byte transmitting
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(10);
}

static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
  uint8_t retval;
  mpu_write_reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(10);
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(10);
  retval = mpu_read_reg(MPU6500_I2C_SLV4_DI);
  //turn off slave4 after read
  mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  return retval;
}

static void mpu_mst_i2c_auto_read_config(uint8_t device_address, uint8_t data)
{
  
}

uint8_t ist8310_init(void)
{
	mpu_write_reg(MPU6500_USER_CTRL, 0x30);
	delay_ms(10);	
	mpu_write_reg(MPU6500_I2C_MST_CTRL, 0x0d);
	delay_ms(10);	
	mpu_write_reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
	delay_ms(10);	
	mpu_write_reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
	delay_ms(10);	
///////////////
	ist_reg_write_by_mpu( IST8310_R_CONFB , 0x01);
	delay_ms(10);	
	
	if(IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
		return 1;
	
	ist_reg_write_by_mpu( IST8310_R_CONFB , 0x01);
	delay_ms(10);	

	ist_reg_write_by_mpu( IST8310_R_CONFA , 0x00);
	if(ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
		return 2;
	delay_ms(10);	
	
	ist_reg_write_by_mpu( IST8310_R_CONFB , 0x00);
	if(ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
		return 3;
	delay_ms(10);		
	
	ist_reg_write_by_mpu( IST8310_AVGCNTL , 0x24);
	if(ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
		return 4;
	delay_ms(10);	
	
	ist_reg_write_by_mpu( IST8310_PDCNTL , 0xc0);
	if(ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
		return 5;
	delay_ms(10);		
/////////////
 	mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
	delay_ms(10);	 	
	
	mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
	delay_ms(10);		
	
//	mpu_mst_i2c_auto_read_config(IST8310_ADDRESS , IST8310_R_XL , 0x06);
//	delay_ms(100);		
	return 0;
}

uint8_t ist_buff[6];
float test_yaw1, test_yaw2;
void ist8310_get_data(uint8_t* buff)
{
	ImuSPI5_ReadData(MPU6500_EXT_SENS_DATA_00 , buff ,6);
}

void mpu_get_data(void)
{
	float temp;
	ImuSPI5_ReadData(MPU6500_ACCEL_XOUT_H , mpu_buff ,14);
	
	mpu_data.ax    = mpu_buff[0]  << 8 | mpu_buff[1];
	mpu_data.ay    = mpu_buff[2]  << 8 | mpu_buff[3];
	mpu_data.az    = mpu_buff[4]  << 8 | mpu_buff[5];
	mpu_data.temp  = mpu_buff[6]  << 8 | mpu_buff[7];
 
	mpu_data.gx    =(mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset;
	mpu_data.gy    =(mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset;
	mpu_data.gz    =(mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset;
 	
	ist8310_get_data(ist_buff);
	memcpy(&mpu_data.mx, ist_buff , 6);
	
	memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	imu.temp = 21 + mpu_data.temp / 333.87f;
	imu.wx   = mpu_data.gx / 16.384f / 57.3f;
	imu.wy   = mpu_data.gy / 16.384f / 57.3f;
	imu.wz   = mpu_data.gz / 16.384f / 57.3f;	
	
	gimbal_speed_yaw   =  imu.wz;
	gimbal_speed_pitch = -imu.wx;
	
//	temp = sqrtf((float)(mpu_data.ax*mpu_data.ax+mpu_data.az*mpu_data.az));
//	temp = atan2((float)mpu_data.ay,temp)*(180/3.14159265);
//	
//	pitch_angle = temp;
 
}


uint8_t MPU_Set_Gyro_LPF(uint8_t data)
{
	//0 250Hz
	//1 184Hz
	//2 92Hz
	//3 41Hz
	//4 20Hz
	//5 10Hz
	//6 5Hz
	//7 3600Hz
  return mpu_write_reg(MPU6500_CONFIG, data);
}

uint8_t mpu_id;

uint8_t mpu_device_init(void)
{
	mpu_write_reg(MPU6500_PWR_MGMT_1, 0x80);
	delay_ms(100);
	
	mpu_write_reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
	delay_ms(100);	
	
	mpu_id = mpu_read_reg(MPU6500_WHO_AM_I);
	
	uint8_t MPU6500_Init_Data[7][2] = {
		{MPU6500_PWR_MGMT_1,           0x03},
		{MPU6500_PWR_MGMT_2,           0x00},
		{MPU6500_CONFIG,               0x04},
		{MPU6500_GYRO_CONFIG,          0x18},		
		{MPU6500_ACCEL_CONFIG,         0x10},		
		{MPU6500_ACCEL_CONFIG_2,       0x04},	
		{MPU6500_USER_CTRL,            0x20},		
	};
	uint8_t i = 0;
	
	for(i = 0 ; i < 7 ; i++)
	{
		mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		delay_ms(1);
	}
	
	ist8310_init();
	mpu_offset_cal();
	
	return 0;
}

void mpu_offset_cal(void)
{
	int i;
	for(i = 0; i < 500; i++)
	{
		ImuSPI5_ReadData(MPU6500_ACCEL_XOUT_H , mpu_buff , 14);
		
		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];
		
		delay_ms(1);
	}
	
	mpu_data.ax_offset = mpu_data.ax_offset / 500.0;
	mpu_data.ay_offset = mpu_data.ay_offset / 500.0;
	mpu_data.az_offset = mpu_data.az_offset / 500.0;	
	
	mpu_data.gx_offset = mpu_data.gx_offset / 500.0;
	mpu_data.gy_offset = mpu_data.gy_offset / 500.0;
	mpu_data.gz_offset = mpu_data.gz_offset / 500.0;	
}
	









