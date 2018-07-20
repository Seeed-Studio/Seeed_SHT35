#include "Seeed_SHT35.h"


/*SAMD core*/
#ifdef defined (ARDUINO_SAMD_VARIANT_COMPLIANCE )
  #define SDAPIN  20
  #define SCLPIN  21
  #define RSTPIN  7
  #define SERIAL SerialUSB
#else
  #define SDAPIN  A4
  #define SCLPIN  A5
  #define RSTPIN  2
  #define SERIAL Serial
#endif

SHT35 sensor(SCLPIN);


void setup()
{
    SERIAL.begin(115200);
    delay(10);
    SERIAL.println("serial start!!");
    if(sensor.init())
    {
      SERIAL.println("sensor init failed!!!");
    }
    delay(1000);
}


void loop()
{
     u16 value=0;
    u8 data[6]={0};
    float temp,hum;
    // sensor.read_reg_status(&value);
    // SERIAL.print("status =");
    // SERIAL.println(value,HEX);
    sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH,&temp,&hum);
    SERIAL.print("temp =");
    SERIAL.println(temp);

    SERIAL.print("hum =");
    SERIAL.println(hum);

    delay(1000);
}




