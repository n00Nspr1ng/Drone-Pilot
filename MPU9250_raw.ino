// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

//void get_one_sample_date_mxyz();
//void getAccel_Data(void);
//void getGyro_Data(void);
//void getCompass_Data(void);
//void getCompassDate_calibrated ();
//
//uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

//float heading;
//float tiltheading;
//
//float Axyz[3];
//float Mxyz[3];

#define sample_num_mdate  5000

//volatile float mx_sample[3];
//volatile float my_sample[3];
//volatile float mz_sample[3];
//
//static float mx_centre = 0;
//static float my_centre = 0;
//static float mz_centre = 0;
//
//volatile int mx_max = 0;
//volatile int my_max = 0;
//volatile int mz_max = 0;
//
//volatile int mx_min = 0;
//volatile int my_min = 0;
//volatile int mz_min = 0;
//
//float temperature;
//float pressure;
//float atm;
//float altitude;


void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}
