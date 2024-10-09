#include "ICM42688_driver.h"
#include "stdio.h"
#include "bsp_dwt.h"
#include "ICM42688_Middleware.h"
#include "ICM42688_reg.h"

static float accSensitivity = 0.244f; // 加速度的最小分辨率 mg/LSB
static float gyroSensitivity = 32.8f; // 陀螺仪的最小分辨率

static void ICM42688_write_single_reg(uint8_t reg, uint8_t data);
static void ICM42688_read_single_reg(uint8_t reg, uint8_t *return_data);
static void ICM42688_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#define ICM42688DelayMs(ms) DWT_Delay(ms/1000.0f);

#define ICM_SPI_CS_LOW() HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET)

#define ICM_SPI_CS_HIGH() HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET)

#define ICM42688_WRITE_SINGLE_REG(reg, data)      \
    {                                             \
        ICM_SPI_CS_LOW();                         \
        ICM42688_write_single_reg((reg), (data)); \
        ICM_SPI_CS_HIGH();                        \
    }

#define ICM42688_READ_SINGLE_REG(reg, data)      \
    {                                            \
        ICM_SPI_CS_LOW();                        \
				ICM42688_read_single_reg(reg,&(data));      \
        ICM_SPI_CS_HIGH();                       \
    }

#define ICM42688_READ_MULI_REG(reg, data, len) \
    {                                          \
    ICM_SPI_CS_LOW();                          \
    ICM42688_read_write_byte((reg) | 0x80);    \
    ICM42688_read_muli_reg(reg, data , len);    \
    ICM_SPI_CS_HIGH();                          \
    }

int16_t ICM42688_init(void)
{
    uint8_t reg_val = 0;
    /* 读取 who am i 寄存器 */
    ICM42688_READ_SINGLE_REG(ICM42688_WHO_AM_I, reg_val);
    // printf("reg_val:%d\n",reg_val);
    ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0);    // 设置bank 0区域寄存器
    ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x01); // 软复位传感器
    //DWT_Delay(0.01);
    ICM42688DelayMs(100);

    if (reg_val == ICM42688_ID)
    {

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 1);    // 设置bank 1区域寄存器
        ICM42688_WRITE_SINGLE_REG(ICM42688_INTF_CONFIG4, 0x02); // 设置为4线SPI通信

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0);   // 设置bank 0区域寄存器
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG, 0x40); // Stream-to-FIFO Mode(page63)

        ICM42688_READ_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_SOURCE0, 0x00);
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG2, 0x00); // watermark
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG3, 0x02); // watermark
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_CONFIG, 0x36);

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);
        reg_val |= (1 << 2); // FIFO_THS_INT1_ENABLE
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);

        bsp_Icm42688GetAres(AFS_8G);
        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_ACCEL_CONFIG0, reg_val); // page74
        reg_val |= (AFS_8G << 5);                                  // 量程 ±8g
        reg_val |= (AODR_1000Hz);                                  // 输出速率 50HZ
        ICM42688_WRITE_SINGLE_REG(ICM42688_ACCEL_CONFIG0, reg_val);

        bsp_Icm42688GetGres(GFS_1000DPS);
        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_GYRO_CONFIG0, reg_val); // page73
        reg_val |= (GFS_1000DPS << 5);                            // 量程 ±1000dps
        reg_val |= (AODR_1000Hz);                                 // 输出速率 50HZ
        ICM42688_WRITE_SINGLE_REG(ICM42688_GYRO_CONFIG0, reg_val);

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_PWR_MGMT0, reg_val); // 读取PWR—MGMT0当前寄存器的值(page72)
        reg_val &= ~(1 << 5);                                  // 使能温度测量
        reg_val |= ((3) << 2);                                 // 设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);                                        // 设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        ICM42688_WRITE_SINGLE_REG(ICM42688_PWR_MGMT0, reg_val);
        
        ICM42688DelayMs(1); // 操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

        return 0;
    }
		else
			
    return -1;
}

/*******************************************************************************
* 名    称： bsp_IcmGetRawData
* 功    能： 读取Icm42688加速度陀螺仪数据
* 入口参数： 六轴
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62,63
*******************************************************************************/
void bsp_IcmGetRawData(IMU_Data_t ICM42688)
{
    uint8_t buffer[12] = {0};

    ICM42688_READ_MULI_REG(ICM42688_ACCEL_DATA_X1, buffer, 12);

    ICM42688.Accel[0]= ((uint16_t)buffer[0] << 8)  | buffer[1];
    ICM42688.Accel[1]  = ((uint16_t)buffer[2] << 8)  | buffer[3];
    ICM42688.Accel[2]  = ((uint16_t)buffer[4] << 8)  | buffer[5];
    ICM42688.Gyro[0] = ((uint16_t)buffer[6] << 8)  | buffer[7];
    ICM42688.Gyro[1] = ((uint16_t)buffer[8] << 8)  | buffer[9];
    ICM42688.Gyro[2] = ((uint16_t)buffer[10] << 8) | buffer[11];


     ICM42688.Accel[0] = (int16_t)( ICM42688.Accel[0]* accSensitivity);
     ICM42688.Accel[1] = (int16_t)( ICM42688.Accel[1] * accSensitivity);
     ICM42688.Accel[2] = (int16_t)( ICM42688.Accel[2] * accSensitivity);

    ICM42688.Gyro[0] = (int16_t)(ICM42688.Gyro[0] * gyroSensitivity);
    ICM42688.Gyro[1] = (int16_t)(ICM42688.Gyro[1] * gyroSensitivity);
    ICM42688.Gyro[2] = (int16_t)(ICM42688.Gyro[2] * gyroSensitivity);

}



static void ICM42688_write_single_reg(uint8_t reg, uint8_t data)
{
    ICM42688_read_write_byte(reg);
    ICM42688_read_write_byte(data);
}

/*******************************************************************************
 * 名    称： ICM42688_read_single_reg
 * 功    能： 读取单个寄存器的值
 * 入口参数： reg: 寄存器地址
 * 出口参数： 当前寄存器地址的值
 * 作　　者： zgh
 * 创建日期： 2024-10
 * 修    改：
 * 修改日期：
 * 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
 *******************************************************************************/
static void ICM42688_read_single_reg(uint8_t reg, uint8_t *return_data)
{
		ICM42688_read_write_byte(reg | 0x80);//表示读取，最高位为读写位
    *return_data = ICM42688_read_write_byte(0);
}

/*******************************************************************************
 * 名    称： ICM42688_read_single_regs
 * 功    能： 连续读取多个寄存器的值
 * 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
 * 出口参数： 无
 * 作　　者： zgh
 * 创建日期： 2024-10
 * 修    改：
 * 修改日期：
 * 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
 *******************************************************************************/
static void ICM42688_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;
    //    ICM42688_read_write_byte(reg | 0x80);
    for (i = 0; i < len; i++)
    {

        *buf = ICM42688_read_write_byte(0x55);
        buf++;
    }
}

float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch (Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch (Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}
