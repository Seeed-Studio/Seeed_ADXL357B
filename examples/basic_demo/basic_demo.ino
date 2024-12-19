#include "Seeed_adxl357b.h"



#if defined(ARDUINO_ARCH_AVR)
    #pragma message("Defined architecture for ARDUINO_ARCH_AVR.")
    #define SERIAL Serial
#elif defined(SEEED_XIAO_M0)
    #pragma message("Defined architecture for SEEED_XIAO_M0.")
    #define SERIAL Serial
#elif defined(ARDUINO_ARCH_SAM)
    #pragma message("Defined architecture for ARDUINO_ARCH_SAM.")
    #define SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_SAMD)
    #pragma message("Defined architecture for ARDUINO_ARCH_SAMD.")
    #define SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_STM32F4)
    #pragma message("Defined architecture for ARDUINO_ARCH_STM32F4.")
    #define SERIAL SerialUSB
#else
    #pragma message("Not found any architecture.")
    #define SERIAL Serial
#endif


#define CALI_BUF_LEN           15
#define CALI_INTERVAL_TIME     250
int32_t cali_buf[3][CALI_BUF_LEN];
int32_t cali_data[3];

float factory;

Adxl357b  adxl357b;


int32_t deal_cali_buf(int32_t* buf) {
    int32_t cali_val = 0;

    for (int i = 0; i < CALI_BUF_LEN; i++) {
        cali_val += buf[i];
    }
    cali_val = cali_val / CALI_BUF_LEN;
    return (int32_t)cali_val;
}


void calibration(void) {
    int32_t x;
    SERIAL.println("Please Place the module horizontally!");
    delay(1000);
    SERIAL.println("Start calibration........");

    for (int i = 0; i < CALI_BUF_LEN; i++) {
        if (adxl357b.checkDataReady()) {
            if (adxl357b.readXYZAxisResultData(cali_buf[0][i], cali_buf[1][i], cali_buf[2][i])) {
            }
        }
        delay(CALI_INTERVAL_TIME);
        // SERIAL.print('.');
    }
    // SERIAL.println('.');
    for (int i = 0; i < 3; i++) {
        cali_data[i] =  deal_cali_buf(cali_buf[i]);
        SERIAL.println(cali_data[i]);
    }
    x = (((cali_data[2] - cali_data[0]) + (cali_data[2] - cali_data[1])) / 2);
    factory = 1.0 / (float)x;
    // SERIAL.println(x);
    SERIAL.println("Calibration OK!!");
}






void setup(void) {
    uint8_t value = 0;
    float t;

    SERIAL.begin(115200);
    if (adxl357b.begin()) {
        SERIAL.println("Can't detect ADXL357B device .");
        while (1);
    }
    SERIAL.println("Init OK!");
    /*Set full scale range to ±40g*/
    adxl357b.setAdxlRange(FOURTY_G);
    /*Switch standby mode to measurement mode.*/
    adxl357b.setPowerCtr(0);
    delay(100);
    /*Read Uncalibration temperature.*/
    adxl357b.readTemperature(t);

    SERIAL.print("Uncalibration  temp = ");
    SERIAL.println(t);
    /**/
    calibration();

}


void loop(void) {
    int32_t x, y, z;
    uint8_t entry = 0;
    if (adxl357b.checkDataReady()) {
        if (adxl357b.readXYZAxisResultData(x, y, z)) {
            SERIAL.println("Get data failed!");
        }
        SERIAL.print("X axis = ");
        SERIAL.print(x * factory);
        SERIAL.println('g');
        SERIAL.print("Y axis = ");
        SERIAL.print(y * factory);
        SERIAL.println('g');
        SERIAL.print("Z axis = ");
        SERIAL.print(z * factory);
        SERIAL.println('g');

    }
    delay(100);
}




