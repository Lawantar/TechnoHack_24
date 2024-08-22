/*
 * mpu9250.c
 *
 *  Created on: Dec 6, 2021
 *      Author: timagr615
 */

#include "main.h"
#include "MPU9250.h"
#include "math.h"


extern I2C_HandleTypeDef hi2c1;

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer

float GyroMeasError = PI * (40.0 / 180.0);     // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0 / 180.0);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0 / 4.0) * PI * (40.0 / 180.0);
float zeta = sqrt(3.0f / 4.0f) * PI * (0.0 / 180.0);
double deltaT = 0.0;
uint32_t newTime = 0;
uint32_t oldTime = 0;

uint8_t setupMPU(MPU9250_t *MPU9250, uint8_t addr) {
        // addr should be valid for MPU
        if ((addr>>1 < MPU9250_DEFAULT_ADDRESS) || (addr>>1 > MPU9250_DEFAULT_ADDRESS + 7)) {
            return 0;
        }
        //mpu_i2c_addr = addr;


        if (isConnectedMPU9250(MPU9250)==1) {
        	MPU9250_Init(MPU9250);
            if (isConnectedAK8963(MPU9250)==1)
                initAK8963(MPU9250);
            else {
            	MPU9250->has_connected = 0;
                return 0;
            }
        } else {
        	MPU9250->has_connected = 0;
            return 0;
        }
        MPU9250->has_connected = 1;
        return 1;
    }


void calibrate(MPU9250_t *MPU9250){
	calibrate_acc_gyro_impl(MPU9250);
	calibrate_mag_impl(MPU9250);
}

void sleep(uint8_t b){
	uint8_t c = readByte(MPU9250_ADDRESS, PWR_MGMT_1);  // read the value, change sleep bit to match b, write byte back to register
	        if (b==1) {
	            c = c | 0x40;  // sets the sleep bit
	        } else {
	            c = c & 0xBF;  // mask 1011111 keeps all the previous bits
	        }
	        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, c);
}

uint8_t isConnectedMPU9250(){
	uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	uint8_t b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
	b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
	b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
	return b;
}
uint8_t isConnectedAK8963(){
	uint8_t c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	return (c == AK8963_WHOAMI_DEFAULT_VALUE);
}
uint8_t MPUisConnected(MPU9250_t *MPU9250){
	MPU9250 -> has_connected = isConnectedMPU9250() && isConnectedAK8963();
	return MPU9250 -> has_connected;
}

uint8_t MPUisSleeping(){
	uint8_t c = readByte(MPU9250_ADDRESS, PWR_MGMT_1);
	return (c & 0x40) == 0x40;
}
uint8_t MPUavailable(MPU9250_t *MPU9250){
	return MPU9250 -> has_connected && (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01);
}

uint8_t updateMPU(MPU9250_t *MPU9250){
	if (!MPUavailable(MPU9250)==1){
		return 0;
	}

	update_accel_gyro(MPU9250);
	update_mag(MPU9250);
    update_rpy(MPU9250, MPU9250->q[0], MPU9250->q[1], MPU9250->q[2], MPU9250->q[3]);

	return 1;
}

float getRoll(MPU9250_t *MPU9250) { return MPU9250->rpy[0]; }
float getPitch(MPU9250_t *MPU9250) { return MPU9250->rpy[1]; }
float getYaw(MPU9250_t *MPU9250) { return MPU9250->rpy[2]; }

float getEulerX(MPU9250_t *MPU9250) { return MPU9250->rpy[0]; }
float getEulerY(MPU9250_t *MPU9250) { return -MPU9250->rpy[1]; }
float getEulerZ(MPU9250_t *MPU9250) { return -MPU9250->rpy[2]; }

float getQuaternionX(MPU9250_t *MPU9250) { return MPU9250->q[1]; }
float getQuaternionY(MPU9250_t *MPU9250) { return MPU9250->q[2]; }
float getQuaternionZ(MPU9250_t *MPU9250) { return MPU9250->q[3]; }
float getQuaternionW(MPU9250_t *MPU9250) { return MPU9250->q[0]; }

float getAcc(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->a[i] : 0.f; }
float getGyro(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->g[i] : 0.f; }
float getMag(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->m[i] : 0.f; }
float getLinearAcc(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->lin_acc[i] : 0.f; }

float getAccX(MPU9250_t *MPU9250) { return MPU9250->a[0]; }
float getAccY(MPU9250_t *MPU9250) { return MPU9250->a[1]; }
float getAccZ(MPU9250_t *MPU9250) { return MPU9250->a[2]; }
float getGyroX(MPU9250_t *MPU9250) { return MPU9250->g[0]; }
float getGyroY(MPU9250_t *MPU9250) { return MPU9250->g[1]; }
float getGyroZ(MPU9250_t *MPU9250) { return MPU9250->g[2]; }
float getMagX(MPU9250_t *MPU9250) { return MPU9250->m[0]; }
float getMagY(MPU9250_t *MPU9250) { return MPU9250->m[1]; }
float getMagZ(MPU9250_t *MPU9250) { return MPU9250->m[2]; }
float getLinearAccX(MPU9250_t *MPU9250) { return MPU9250->lin_acc[0]; }
float getLinearAccY(MPU9250_t *MPU9250) { return MPU9250->lin_acc[1]; }
float getLinearAccZ(MPU9250_t *MPU9250) { return MPU9250->lin_acc[2]; }
float getAccBias(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->acc_bias[i] : 0.f; }
float getGyroBias(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->gyro_bias[i] : 0.f; }
float getMagBias(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->mag_bias[i] : 0.f; }
float getMagScale(MPU9250_t *MPU9250, const uint8_t i) { return (i < 3) ? MPU9250->mag_scale[i] : 0.f; }
float getAccBiasX(MPU9250_t *MPU9250) { return MPU9250->acc_bias[0]; }
float getAccBiasY(MPU9250_t *MPU9250) { return MPU9250->acc_bias[1]; }
float getAccBiasZ(MPU9250_t *MPU9250) { return MPU9250->acc_bias[2]; }
float getGyroBiasX(MPU9250_t *MPU9250) { return MPU9250->gyro_bias[0]; }
float getGyroBiasY(MPU9250_t *MPU9250) { return MPU9250->gyro_bias[1]; }
float getGyroBiasZ(MPU9250_t *MPU9250) { return MPU9250->gyro_bias[2]; }
float getMagBiasX(MPU9250_t *MPU9250) { return MPU9250->mag_bias[0]; }
float getMagBiasY(MPU9250_t *MPU9250) { return MPU9250->mag_bias[1]; }
float getMagBiasZ(MPU9250_t *MPU9250) { return MPU9250->mag_bias[2]; }
float getMagScaleX(MPU9250_t *MPU9250) { return MPU9250->mag_scale[0]; }
float getMagScaleY(MPU9250_t *MPU9250) { return MPU9250->mag_scale[1]; }
float getMagScaleZ(MPU9250_t *MPU9250) { return MPU9250->mag_scale[2]; }
float getTemperature(MPU9250_t *MPU9250) { return MPU9250->temperature; }

void setAccBias(MPU9250_t *MPU9250, const float x, const float y, const float z) {
	MPU9250->acc_bias[0] = x;
	MPU9250->acc_bias[1] = y;
	MPU9250->acc_bias[2] = z;
       write_accel_offset(MPU9250);
   }
void setGyroBias(MPU9250_t *MPU9250, const float x, const float y, const float z) {
	   MPU9250->gyro_bias[0] = x;
	   MPU9250->gyro_bias[1] = y;
	   MPU9250->gyro_bias[2] = z;
       write_gyro_offset(MPU9250);
   }
void setMagBias(MPU9250_t *MPU9250, const float x, const float y, const float z) {
	MPU9250->mag_bias[0] = x;
	MPU9250->mag_bias[1] = y;
	MPU9250->mag_bias[2] = z;
   }
void setMagScale(MPU9250_t *MPU9250, const float x, const float y, const float z) {
	MPU9250->mag_scale[0] = x;
	MPU9250->mag_scale[1] = y;
	MPU9250->mag_scale[2] = z;
   }

void setMagneticDeclination(MPU9250_t *MPU9250, const float d) {
	MPU9250->magnetic_declination = d;
}

void update_rpy(MPU9250_t *MPU9250, float qw, float qx, float qy, float qz){
	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	        // In this coordinate system, the positive z-axis is down toward Earth.
	        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
	        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	        float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
	        a12 = 2.0f * (qx * qy + qw * qz);
	        a22 = qw * qw + qx * qx - qy * qy - qz * qz;
	        a31 = 2.0f * (qw * qx + qy * qz);
	        a32 = 2.0f * (qx * qz - qw * qy);
	        a33 = qw * qw - qx * qx - qy * qy + qz * qz;
	        MPU9250->rpy[0] = atan2f(a31, a33);
	        MPU9250->rpy[1] = -asinf(a32);
	        MPU9250->rpy[2] = atan2f(a12, a22);
	        MPU9250->rpy[0] *= 180.0f / PI;
	        MPU9250->rpy[1] *= 180.0f / PI;
	        MPU9250->rpy[2] *= 180.0f / PI;
	        MPU9250->rpy[2] += MPU9250->magnetic_declination;
	        if (MPU9250->rpy[2] >= +180.f)
	        	MPU9250->rpy[2] -= 360.f;
	        else if (MPU9250->rpy[2] < -180.f)
	        	MPU9250->rpy[2] += 360.f;

	        MPU9250->lin_acc[0] = MPU9250->a[0] + a31;
	        MPU9250->lin_acc[1] = MPU9250->a[1] + a32;
	        MPU9250->lin_acc[2] = MPU9250->a[2] - a33;
	        //char str[] = "IN UPDATE rpy \n\r";
	        //HAL_UART_Transmit(&huart1, str, strlen((char *)str), 0xFFFF);
}

void update_accel_gyro(MPU9250_t *MPU9250) {
        int16_t raw_acc_gyro_data[7];        // used to read all 14 bytes at once from the MPU9250 accel/gyro
        readAccelGyro(raw_acc_gyro_data);  // INT cleared on any read

        // Now we'll calculate the accleration value into actual g's
        MPU9250->a[0] = (float)raw_acc_gyro_data[0] * MPU9250->acc_resolution;  // get actual g value, this depends on scale being set
        MPU9250->a[1] = (float)raw_acc_gyro_data[1] * MPU9250->acc_resolution;
        MPU9250->a[2] = (float)raw_acc_gyro_data[2] * MPU9250->acc_resolution;

        MPU9250->temperature_count = raw_acc_gyro_data[3];                  // Read the adc values
        MPU9250->temperature = ((float)MPU9250->temperature_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade

        // Calculate the gyro value into actual degrees per second
        MPU9250->g[0] = (float)raw_acc_gyro_data[4] * MPU9250->gyro_resolution;  // get actual gyro value, this depends on scale being set
        MPU9250->g[1] = (float)raw_acc_gyro_data[5] * MPU9250->gyro_resolution;
        MPU9250->g[2] = (float)raw_acc_gyro_data[6] * MPU9250->gyro_resolution;
    }

void update_mag(MPU9250_t *MPU9250) {
        int16_t mag_count[3] = {0, 0, 0};  // Stores the 16-bit signed magnetometer sensor output

        // Read the x/y/z adc values
        if (readMag(mag_count)==1) {
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            // mag_bias is calcurated in 16BITS
            float bias_to_current_bits = MPU9250->mag_resolution / (10.*4912./32760.0);
            MPU9250->m[0] = (float)(mag_count[0] * MPU9250->mag_resolution * MPU9250->mag_bias_factory[0] - MPU9250->mag_bias[0] * bias_to_current_bits) * MPU9250->mag_scale[0];  // get actual magnetometer value, this depends on scale being set
            MPU9250->m[1] = (float)(mag_count[1] * MPU9250->mag_resolution * MPU9250->mag_bias_factory[1] - MPU9250->mag_bias[1] * bias_to_current_bits) * MPU9250->mag_scale[1];
            MPU9250->m[2] = (float)(mag_count[2] * MPU9250->mag_resolution * MPU9250->mag_bias_factory[2] - MPU9250->mag_bias[2] * bias_to_current_bits) * MPU9250->mag_scale[2];
        }
    }

void readAccelGyro(int16_t* destination)
{

	uint8_t raw_data[14];                                                 // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &raw_data[0]);             // Read the 14 raw data registers into data array
	destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1];  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
	destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
	destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
	destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
	destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
	destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
}


uint8_t readMag(int16_t* destination)
{
	const uint8_t st1 = readByte(AK8963_ADDRESS, AK8963_ST1);
	if (st1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
	            uint8_t raw_data[7];                                             // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &raw_data[0]);      // Read the six raw data and ST2 registers sequentially into data array
	            if (Mmode == 0x02 || Mmode == 0x04 || Mmode == 0x06) {  // continuous or external trigger read mode
	                if ((st1 & 0x02) != 0)                                       // check if data is not skipped
	                    return 0;                                            // this should be after data reading to clear DRDY register
	            }

	            uint8_t c = raw_data[6];                                         // End data read by reading ST2 register
	            if (!(c & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
	                destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
	                destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
	                destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
	                return 1;
	            }
	        }
	        return 0;
}

void calibrate_acc_gyro_impl(MPU9250_t *MPU9250){
	set_acc_gyro_to_calibration();
	collect_acc_gyro_data_to(MPU9250->acc_bias, MPU9250->gyro_bias);
	write_accel_offset(MPU9250);
	write_gyro_offset(MPU9250);
	HAL_Delay(100);
	MPU9250_Init(MPU9250);
	HAL_Delay(1000);
}
void set_acc_gyro_to_calibration(){
	// reset device
	        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
	        HAL_Delay(100);

	        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	        // else use the internal oscillator, bits 2:0 = 001
	        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	        writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	        HAL_Delay(200);

	        // Configure device for bias calculation
	        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);    // Disable all interrupts
	        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);       // Disable FIFO
	        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);    // Turn on internal clock source
	        writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);  // Disable I2C master
	        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
	        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);     // Reset FIFO and DMP
	        HAL_Delay(15);

	        // Configure MPU6050 gyro and accelerometer for bias calculation
	        writeByte(MPU9250_ADDRESS, CONFIG, 0x01);    // Set low-pass filter to 188 Hz
	        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
	        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

	        // Configure FIFO to capture accelerometer and gyro data for bias calculation
	        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
	        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	        HAL_Delay(40);                                  // accumulate 40 samples in 40 milliseconds = 480 bytes

}
void collect_acc_gyro_data_to(float * a_bias, float * g_bias){
	uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
	        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
	        readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
	        uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	        uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

	        for (uint16_t ii = 0; ii < packet_count; ii++) {
	            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
	            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);              // read data for averaging
	            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
	            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
	            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
	            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
	            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
	            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

	            a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	            a_bias[1] += (float)accel_temp[1];
	            a_bias[2] += (float)accel_temp[2];
	            g_bias[0] += (float)gyro_temp[0];
	            g_bias[1] += (float)gyro_temp[1];
	            g_bias[2] += (float)gyro_temp[2];
	        }
	        a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
	        a_bias[1] /= (float)packet_count;
	        a_bias[2] /= (float)packet_count;
	        g_bias[0] /= (float)packet_count;
	        g_bias[1] /= (float)packet_count;
	        g_bias[2] /= (float)packet_count;

	        if (a_bias[2] > 0L) {
	            a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
	        }  // Remove gravity from the z-axis accelerometer bias calculation
	        else {
	            a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
	        }
}
void write_accel_offset(MPU9250_t *MPU9250){
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	        // the accelerometer biases calculated above must be divided by 8.

	        uint8_t read_data[2] = {0};
	        int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
	        readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
	        acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
	        readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &read_data[0]);
	        acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
	        readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &read_data[0]);
	        acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

	        int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
	        for (int i = 0; i < 3; i++) {
	            if (acc_bias_reg[i] % 2) {
	                mask_bit[i] = 0;
	            }
	            acc_bias_reg[i] -= (int16_t)MPU9250->acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
	            if (mask_bit[i]) {
	                acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
	            } else {
	                acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
	            }
	        }

	        uint8_t write_data[6] = {0};
	        write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
	        write_data[1] = (acc_bias_reg[0]) & 0xFF;
	        write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
	        write_data[3] = (acc_bias_reg[1]) & 0xFF;
	        write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
	        write_data[5] = (acc_bias_reg[2]) & 0xFF;

	        // Push accelerometer biases to hardware registers
	        writeByte(MPU9250_ADDRESS, XA_OFFSET_H, write_data[0]);
	        writeByte(MPU9250_ADDRESS, XA_OFFSET_L, write_data[1]);
	        writeByte(MPU9250_ADDRESS, YA_OFFSET_H, write_data[2]);
	        writeByte(MPU9250_ADDRESS, YA_OFFSET_L, write_data[3]);
	        writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, write_data[4]);
	        writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, write_data[5]);
}
void write_gyro_offset(MPU9250_t *MPU9250){
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	        uint8_t gyro_offset_data[6] = {0};
	        gyro_offset_data[0] = (-(int16_t)MPU9250->gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	        gyro_offset_data[1] = (-(int16_t)MPU9250->gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
	        gyro_offset_data[2] = (-(int16_t)MPU9250->gyro_bias[1] / 4 >> 8) & 0xFF;
	        gyro_offset_data[3] = (-(int16_t)MPU9250->gyro_bias[1] / 4) & 0xFF;
	        gyro_offset_data[4] = (-(int16_t)MPU9250->gyro_bias[2] / 4 >> 8) & 0xFF;
	        gyro_offset_data[5] = (-(int16_t)MPU9250->gyro_bias[2] / 4) & 0xFF;

	        // Push gyro biases to hardware registers
	        writeByte(MPU9250_ADDRESS, XG_OFFSET_H, gyro_offset_data[0]);
	        writeByte(MPU9250_ADDRESS, XG_OFFSET_L, gyro_offset_data[1]);
	        writeByte(MPU9250_ADDRESS, YG_OFFSET_H, gyro_offset_data[2]);
	        writeByte(MPU9250_ADDRESS, YG_OFFSET_L, gyro_offset_data[3]);
	        writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, gyro_offset_data[4]);
	        writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, gyro_offset_data[5]);
}

void calibrate_mag_impl(MPU9250_t *MPU9250){
	// set MAG_OUTPUT_BITS to maximum to calibrate

	        uint8_t Mscale_cache = Mscale;
	        Mscale = MFS_16BITS;

	        initAK8963(MPU9250);
	        collect_mag_data_to(MPU9250);

	        // restore MAG_OUTPUT_BITS
	        Mscale = Mscale_cache;
	        initAK8963(MPU9250);
}
void collect_mag_data_to(MPU9250_t *MPU9250){
	        HAL_Delay(4000);

	        // shoot for ~fifteen seconds of mag data
	        uint16_t sample_count = 0;
	        if (Mmode == 0x02)
	            sample_count = 128;     // at 8 Hz ODR, new mag data is available every 125 ms
	        else if (Mmode == 0x06)  // in this library, fixed to 100Hz
	            sample_count = 1500;    // at 100 Hz ODR, new mag data is available every 10 ms

	        int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
	        int16_t mag_max[3] = {-32767, -32767, -32767};
	        int16_t mag_min[3] = {32767, 32767, 32767};
	        int16_t mag_temp[3] = {0, 0, 0};
	        for (uint16_t ii = 0; ii < sample_count; ii++) {
	            readMag(mag_temp);  // Read the mag data
	            for (int jj = 0; jj < 3; jj++) {
	                if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
	                if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
	            }
	            if (Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
	            if (Mmode == 0x06) HAL_Delay(12);   // at 100 Hz ODR, new mag data is available every 10 ms
	        }


	        // Get hard iron correction
	        bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	        bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	        bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	        float bias_resolution = Mscale;
	        MPU9250->mag_bias[0] = (float)bias[0] * bias_resolution * MPU9250->mag_bias_factory[0];  // save mag biases in G for main program
	        MPU9250->mag_bias[1] = (float)bias[1] * bias_resolution * MPU9250->mag_bias_factory[1];
	        MPU9250->mag_bias[2] = (float)bias[2] * bias_resolution * MPU9250->mag_bias_factory[2];

	        // Get soft iron correction estimate
	        //*** multiplication by mag_bias_factory added in accordance with the following comment
	        //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
	        scale[0] = (float)(mag_max[0] - mag_min[0]) * MPU9250->mag_bias_factory[0] / 2;  // get average x axis max chord length in counts
	        scale[1] = (float)(mag_max[1] - mag_min[1]) * MPU9250->mag_bias_factory[1] / 2;  // get average y axis max chord length in counts
	        scale[2] = (float)(mag_max[2] - mag_min[2]) * MPU9250->mag_bias_factory[2] / 2;  // get average z axis max chord length in counts

	        float avg_rad = scale[0] + scale[1] + scale[2];
	        avg_rad /= 3.0;

	        MPU9250->mag_scale[0] = avg_rad / ((float)scale[0]);
	        MPU9250->mag_scale[1] = avg_rad / ((float)scale[1]);
	        MPU9250->mag_scale[2] = avg_rad / ((float)scale[2]);
}



void setFilterIterations(MPU9250_t *MPU9250, const size_t n) {
        if (n > 0) MPU9250->n_filter_iter = n;
    }


void initAK8963(MPU9250_t *MPU9250)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  HAL_Delay(100);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(100);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  MPU9250->mag_bias_factory[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  MPU9250->mag_bias_factory[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
  MPU9250->mag_bias_factory[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  HAL_Delay(100);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  HAL_Delay(100);
}

void MPU9250_Init(MPU9250_t *MPU9250){
	// Initialize MPU9250 device
	 // wake up device
	getAres(MPU9250);
	getGres(MPU9250);
	getMres(MPU9250);
	// reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	HAL_Delay(100);

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	HAL_Delay(100);


	 // Configure Gyro and Accelerometer
	 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	 // Set gyroscope full scale range
	 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	  uint8_t c =  readByte(MPU9250_ADDRESS, GYRO_CONFIG);
	  c = c & ~0xE0;                                     // Clear self-test bits [7:5]
	  c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
	  c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
	  c = c | (Gscale << 3);       // Set full scale range for the gyro
	  c = c | (0x03 & 0x03);   // Set Fchoice for the gyro
	  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Set full scale range for the gyro

	 // Set accelerometer configuration
	  c =  readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
	  c = c & ~0xE0;                                 // Clear self-test bits [7:5]
	  c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
	  c = c | (Ascale << 3);  // Set full scale range for the accelerometer
	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Set full scale range for the accelerometer

	 // Set accelerometer sample rate configuration
	 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	  c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	  c = c | (~(0x01 << 3) & 0x08);    // Set accel_fchoice_b to 1
	  c = c | (0x03 & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	  // Configure Interrupts and Bypass Enable
	  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	  // can join the I2C bus and all can be controlled by the Arduino as master
	   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt

}

void getMres(MPU9250_t *MPU9250) {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
    	MPU9250->mag_resolution = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
    	MPU9250->mag_resolution = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}


void getGres(MPU9250_t *MPU9250) {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          MPU9250->gyro_resolution = 250.0/32768.0;
          break;
    case GFS_500DPS:
    	MPU9250->gyro_resolution = 500.0/32768.0;
          break;
    case GFS_1000DPS:
    	MPU9250->gyro_resolution = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
    	MPU9250->gyro_resolution = 2000.0/32768.0;
          break;
  }
}


void getAres(MPU9250_t *MPU9250) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
    	MPU9250->acc_resolution = 2.0/32768.0;
          break;
    case AFS_4G:
    	MPU9250->acc_resolution = 4.0/32768.0;
          break;
    case AFS_8G:
    	MPU9250->acc_resolution = 8.0/32768.0;
          break;
    case AFS_16G:
    	MPU9250->acc_resolution = 16.0/32768.0;
          break;
  }
}


char readByte(uint8_t I2C_ADDRESS, uint8_t RegAddr){
	char data[1];
	char data_write[1];
	data_write[0] = RegAddr;

	//Откуда будем считывать данные

	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, data_write, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)I2C_ADDRESS, data, (uint16_t)1, (uint16_t)100);

	return data[0];

}
void readBytes(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t count, uint8_t * dest)
{
	char data[14];
	char data_write[1];
	data_write[0] = RegAddr;
	//Откуда будем считывать данные

	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, data_write, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(I2C_ADDRESS), data, count, (uint16_t)100);
	for(int ii = 0; ii < count; ii++) {
			dest[ii] = data[ii];
		}

}

void writeByte(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t data){
	char data_write[2];
	data_write[0] = RegAddr;
	data_write[1] = data;

	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, data_write, 2, 100);
}

void MPU9250SetDefault(MPU9250_t *MPU9250){
	MPU9250 -> acc_resolution = 0.;                // scale resolutions per LSB for the sensors
	MPU9250 -> gyro_resolution = 0.;               // scale resolutions per LSB for the sensors
	MPU9250 -> mag_resolution = 0.;                // scale resolutions per LSB for the sensors

		// Calibration Parameters
	MPU9250 -> acc_bias[0] = 0.;   // acc calibration value in ACCEL_FS_SEL: 2g
	MPU9250 -> acc_bias[1] = 0.;
	MPU9250 -> acc_bias[2] = 0.;
	MPU9250 -> gyro_bias[0] = 0.;  // gyro calibration value in GYRO_FS_SEL: 250dps
	MPU9250 -> gyro_bias[1] = 0.;
	MPU9250 -> gyro_bias[2] = 0.;
	MPU9250 -> mag_bias_factory[0] = 0.;
	MPU9250 -> mag_bias_factory[1] = 0.;
	MPU9250 -> mag_bias_factory[2] = 0.;

	MPU9250 -> mag_bias[0] = 0.;  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
	MPU9250 -> mag_bias[1] = 0.;
	MPU9250 -> mag_bias[2] = 0.;
	MPU9250 -> mag_scale[0] = 1.;
	MPU9250 -> mag_scale[1] = 1.;
	MPU9250 -> mag_scale[2] = 1.;
	MPU9250 -> magnetic_declination  = 10.91;  // Moscow, 24th June

		    // Temperature
	MPU9250 -> temperature_count = 0.;  // temperature raw count output
	MPU9250 -> temperature = 0.;        // Stores the real internal chip temperature in degrees Celsius

		// Self Test
	MPU9250 -> self_test_result[0] = 0.;  // holds results of gyro and accelerometer self test
	MPU9250 -> self_test_result[1] = 0.;
	MPU9250 -> self_test_result[2] = 0.;
	MPU9250 -> self_test_result[3] = 0.;
	MPU9250 -> self_test_result[4] = 0.;
	MPU9250 -> self_test_result[5] = 0.;

		// IMU Data
	MPU9250 -> a[0] = 0.;
	MPU9250 -> a[1] = 0.;
	MPU9250 -> a[2] = 0.;
	MPU9250 -> g[0] = 0.;
	MPU9250 -> g[1] = 0.;
	MPU9250 -> g[2] = 0.;
	MPU9250 -> m[0] = 0.;
	MPU9250 -> m[1] = 0.;
	MPU9250 -> m[2] = 0.;
	MPU9250 -> q[0] = 1.;  // vector to hold quaternion
	MPU9250 -> q[1] = 0.;
	MPU9250 -> q[2] = 0.;
	MPU9250 -> q[3] = 0.;
	MPU9250 -> rpy[0] = 0.;
	MPU9250 -> rpy[1] = 0.;
	MPU9250 -> rpy[2] = 0.;
	MPU9250 -> lin_acc[0] = 0.;
	MPU9250 -> lin_acc[1] = 0.;
	MPU9250 -> lin_acc[2] = 0.;

	MPU9250 -> has_connected = 0;
	MPU9250 -> n_filter_iter = 1;

}
