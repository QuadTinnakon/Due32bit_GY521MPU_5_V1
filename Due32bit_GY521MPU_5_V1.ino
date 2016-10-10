/*
 Test By tinnakon kheowree  project 
 tinnakon_za@hotmail.com
 tinnakonza@gmail.com
 http://quad3d-tin.lnwshop.com/
 https://www.facebook.com/tinnakonza

 27/03/2559     write Due32bit_GY521MPU_5_V1

support : Arduino 1.5.8   Arduino Due 32 bit  , MPU6050  MS5611
• Atmel SAM3X8E ARM Cortex-M3 CPU 32-bit a 84 MHz clock, ARM core microcontroller
• MPU6050 Gyro Accelerometer //I2C 400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• MS5611 Barometer//SPI
• HMC5883L Magnetometer //I2C_BYPASS ,I2C 400kHz
• GPS NEO-7N //

Quad-X
pin 9 FRONTL  M1CW        M2CCW  FRONTR pin 8
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
pin 6 motor_BackL  M4 CCW      M3 CW  motor_BackR  pin 7

----------rx-----------  
A8 = PPM 8 CH
 */
#include <Arduino.h>
#include "Wire_due32.h"
#include "SPI_sam.h"
#include "AP_Baro_MS5611.h"
#include "Aconfigsam3x8e.h"
#include "multi_rx_sam3x8e.h"
#include "mpu6050sam3x8e.h"
#include "ahrs_tinsam3x8e.h"
#include "GPSNEO6N_multi.h"
#include "Control_PPIDsam3x8e.h"
#include "motorX4sam3x8e.h"
#include "Ultrasonic04.h"
#include "Kalman_ObserverTin.h"
#include "AnalogSource_Arduino.h"
////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);//115200
  //Serial1.begin(57600);
  Serial.print("Due32bit_GY521MPU_5_V1XXX");Serial.println("\n");
  GPS_multiInt();
  pinMode(13, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(Pin_Laser, OUTPUT);
  pinMode(Pin_LED_B, OUTPUT);
  pinMode(Pin_LED_G, OUTPUT);
  pinMode(Pin_LED_R, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(Pin_Laser, HIGH);
  digitalWrite(Pin_LED_B, LOW);
  digitalWrite(Pin_LED_G, LOW);
  digitalWrite(Pin_LED_R, LOW);
  configureReceiver();
  motor_initialize();
  ESC_calibration();
  delay(10);
  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  Wire1.begin();
  Wire1.setClock(400000);//400000
  delay(10);
  mpu6050_initialize();
  delay(30); //GYROSCOPE START-UP TIME 30 ms
  MagHMC5883Int();
  Serial.print("HMC5883_initialize");Serial.print("\n");
  delay(10);
  SPI.begin();
  //SPI.setClockDivider(SPI_CLOCK_DIV32);     // 500khz for debugging, increase later
  delay(10);
  //baro.init(MS561101BA_ADDR_CSB_LOW);
  baro.init();
  //baro.calibrate();
  //Serial.print("MS5611_initialize");Serial.print("\n");
  delay(10);
  UltrasonicInt();
  delay(10);
  Serial.print("Read Sensor");Serial.println("\t");
     for(int i=0; i<100; i++) 
    {
     mpu6050_readGyroSum();
     mpu6050_readAccelSum();
     Mag5883Read();
     //temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
     //presser = baro.getPressure(MS561101BA_OSR_4096);
     baro._update(micros());
     baro.read();
     presser = baro.get_pressure();
     temperaturetr = baro.get_temperature();
     pushAvg(presser, temperaturetr);
     delay(10);
    }
    presserf = getAvg(movavg_buff, MOVAVG_SIZE);
    temperaturetrf = getAvg(movavg_buffT, MOVAVG_SIZE);
    sea_press = presserf - 0.12;// + 0.09 presser 1007.25   1003.52
    Serial.print("presser ");Serial.print(sea_press);
    Serial.print(" Temperature ");Serial.println(temperaturetrf);
    digitalWrite(Pin_LED_B, HIGH);
    digitalWrite(Pin_LED_G, HIGH);
    digitalWrite(Pin_LED_R, HIGH);
    mpu6050_Get_accel();
    mpu6050_Get_gyro();
    delay(10);
    sensor_Calibrate();//sensor.h
    ahrs_initialize();//ahrs.h
    RC_Calibrate();//"multi_rxPPM2560.h"
    setupFourthOrder();//ahrs
  delay(100);
  digitalWrite(13, LOW);
      for(uint8_t i=0;i<=10;i++){
      //GPS_NewData(); 
        while(Serial2.available())
       {
       char byteGPS1=Serial2.read(); 
       GPS_UBLOX_newFrame(byteGPS1);
       }
       GPS_LAT1 = GPS_coord[LAT]/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
       GPS_LON1 = GPS_coord[LON]/10000000.0;
       GPS_LAT_HOME = GPS_LAT1;
       GPS_LON_HOME = GPS_LON1;
       digitalWrite(Pin_LED_B, LOW);
       delay(20);
       digitalWrite(Pin_LED_B, HIGH);
       delay(80);
    }
  digitalWrite(Pin_Laser, LOW);
  digitalWrite(Pin_LED_B, LOW);
  digitalWrite(Pin_LED_G, LOW);
  digitalWrite(Pin_LED_R, LOW);
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop() {
  while(1){
    while(Serial2.available()){ /////GPS///////////////////////////////
     char byteGPS1=Serial2.read(); 
     GPS_UBLOX_newFrame(byteGPS1);
     }//end gps  ///////////////////////////////////////
    Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor 1 kHz/////////
    if(Dt_sensor <= 0){Dt_sensor = 1001;}
    if(Dt_sensor >= 1000 && gyroSamples < 4)////Collect 2 samples = 1000 us 
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
    }
   Dt_roop = micros() - previousTime;// 200 Hz task loop (5 ms)  , 2500 us = 400 Hz
   if(Dt_roop <= 0){Dt_roop = 5001;}  
   if (Dt_roop >= 5000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
/////get sensor////////////////////////////////////////////////////////////
      mpu6050_Get_accel();
      mpu6050_Get_gyro();
      baro._update(previousTime);
      //digitalWrite(12, HIGH);
      //digitalWrite(12, LOW);
      //////////////////////////////////////////////////////////
  if (frameCounter % TASK_100HZ == 0)// 100 Hz tak
 {
   /*
        ///////////////////Filter FourthOrder ///////////////////////////////////////
    Accel[XAXIS] = AccX;
    Accel[YAXIS] = AccY;
    Accel[ZAXIS] = AccZ;
    for (int axis = XAXIS; axis <= ZAXIS; axis++) {
      filteredAccel[axis] = computeFourthOrder(Accel[axis], &fourthOrder[axis]);//"ahrs_tin.h"
    }
    AccXf = filteredAccel[XAXIS];
    AccYf = filteredAccel[YAXIS];
    AccZf = filteredAccel[ZAXIS];
    */
  //presser = baro.getPressure(MS561101BA_OSR_4096);
  baro.read();
  presser = baro.get_pressure();
  temperaturetr = baro.get_temperature();
  pushAvg(presser, temperaturetr);
 }
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      //AccXf = (AccX + AccX2)/2.0;
      //AccYf = (AccY + AccY2)/2.0;
      //AccZf = (AccZ + AccZ2)/2.0;
      //AccX2 = AccX;AccY2 = AccY;AccZ2 = AccZ;//acc Old1
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1
////////////////Low pass filter/////////////////////////////////
    AccXf = AccXf + (AccX - AccXf)*0.12101;//0.240121 ,0.121  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
    AccYf = AccYf + (AccY - AccYf)*0.12101;//0.240121 ,0.121
    AccZf = AccZf + (AccZ - AccZf)*0.12101;//0.240121 ,0.121
//////////////////////////////////////////////////////////
    ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
//Observer kalman filter//////////////////////////////////
    Observer_kalman_filter();
//Sliding modeControl/////////////////////////////////////
    Control_PPIDRate();//"Control_Slid.h"
//////Out motor///////////
//armed = 1;
    motor_Mix();//"motor.h"
/////////////////////////
    motor_command(); 
////////end Out motor//////
  if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
  //mpu6050_Get_accel();
  computeRC();//multi_rx.h
    if (CH_THR < MINCHECK)  //////ARM and DISARM your Quadrotor///////////////
        {
            if (CH_RUD > MAXCHECK && armed == 0 && abs(ahrs_p) < 10 && abs(ahrs_r) < 10)//+- 10 deg, ARM 
            {
                armed = 1;
                digitalWrite(Pin_LED_R, HIGH);
                Altitude_Ground = Altitude_baro;
                setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
            }
            if (CH_RUD < MINCHECK && armed == 1) //DISARM
            {
                armed = 0;
                digitalWrite(Pin_LED_R, LOW);
                Altitude_Ground = Altitude_baro;
                setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
            }
            if (CH_RUD < MINCHECK && armed == 0 && CH_ELE > MAXCHECK) //Mag_Calibrate
            {
              Mag_Calibrate();//#include "mpu6050.h"
            }
        }//end  ARM and DISARM your helicopter/////////////// 
 }
      if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        {
           Mag5883Read();
           UltrasonicRead();//"Ultrasonic04.h"
          //updateOF();//AP_OPTICALFLOW_ADNS3080
          //Get_pixy();	
          //update_positionA3080(GyroXfMO ,GyroYfMO ,1.0 ,0.0 ,z1_hat);//(float Gyroroll, float Gyropitch, float cos_yaw_x, float sin_yaw_y, float altitude)
          Control_PositionHold();//#include "Control_SlidingPIDsam3x8e.h"
        }
        if (frameCounter % TASK_10HZ == 0)//roop TASK_10HZ
        {
          Chack_Command();
          Automatictakeland();
        }
        if (frameCounter % TASK_5HZ == 0)//GPS_calc TASK_5HZ
        {
           presserf = getAvg(movavg_buff, MOVAVG_SIZE);
           temperaturetrf = getAvg(movavg_buffT, MOVAVG_SIZE);
           presserfF = presserfF + (presserf - presserfF)*0.3852101;//0.9852101 0.785 0.685 0.545 0.345 Low Pass Filter
           Altitude_baro = getAltitude(presserfF,temperaturetrf);//Altitude_Ground
           Altitude_barof = Altitude_baro - Altitude_Ground + Altitude_II;
           baro_vz = (z1_hat - baro_vz_old)/0.2;//G_Dt Diff Baro 5HZ=0.2 s  ,,20 Hz=0.05
           baro_vz_old = z1_hat;
           if(GPS_FIX  == 1){
             //baro_vz = _vel_down/100.0;
           }
           else{
             //baro_vz = baro_vz;
           }
           GPS_LAT1 = GPS_coord[LAT]/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
           GPS_LON1 = GPS_coord[LON]/10000000.0;
           Cal_GPS();//#include "Control_PPIDsam3x8e.h"
           //Control_PositionHold();
           //GPS_distance_m_bearing(GPS_LAT1, GPS_LON1, GPS_LAT_HOME, GPS_LON_HOME, Altitude_hat);
        }
        if (frameCounter % TASK_2HZ == 0)//LED GPS
        {
          if(Status_LED_GPS == LOW && GPS_FIX  == 1)
             {
               Status_LED_GPS = HIGH;
               Counter_LED_GPS++;
               if(Counter_LED_GPS >= GPS_numSat){
               Counter_LED_GPS = 0;
               digitalWrite(Pin_LED_G, HIGH);
               }
               else
               {
               digitalWrite(Pin_LED_G, LOW);
               }
             }
             else
             {
              Status_LED_GPS = LOW;
             }
            digitalWrite(Pin_LED_B, Status_LED_GPS);
        }//end if LED GPS
       if (frameCounter % TASK_NoHZ == 0)//roop print  ,TASK_NoHZ TASK_5HZ  TASK_10HZ
        {
          Voltage = analogRead(A5);
          Ampere = analogRead(A6);
        }
      if (frameCounter % TASK_NoHZ == 0)//roop print  ,TASK_NoHZ TASK_5HZ  TASK_10HZ
        {
            Serial.print(CH_THRf);Serial.print("\t");
            Serial.print(CH_AILf);Serial.print("\t");  
            Serial.print(CH_ELEf);Serial.print("\t");
            Serial.print(CH_RUDf);Serial.print("\t");  
            Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 
            //Serial.print(AUX_3);Serial.print("\t"); 
            //Serial.print(AUX_4);Serial.print("\t"); 
            //Serial.print(failsafeCnt);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 
             
            //Serial.print(MagX1);Serial.print("\t");
            Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagY1);Serial.print("\t");
            Serial.print(MagYf);Serial.print("\t");
            //Serial.print(MagZ1);Serial.print("\t");  
            Serial.print(MagZf);Serial.print("\t");
            
            //Serial.print(c_magnetom_x);Serial.print("\t");
            //Serial.print(c_magnetom_y);Serial.print("\t");
            //Serial.print(c_magnetom_z);Serial.print("\t"); 
            
            //Serial.print(GPS_FIX1);Serial.print("\t");
            //Serial.print(GPS_LAT1,9);Serial.print("\t"); 
            //Serial.print(GPS_LAT1f,9);Serial.print("\t");

            //Serial.print(GPS_LON1,9);Serial.print("\t");
            //Serial.print(GPS_LON1f,9);Serial.print("\t");
            //Serial.print(GPS_LON1f2,9);Serial.print("\t");
            //Serial.print(error_LAT);Serial.print("\t");
            //Serial.print(error_LON);Serial.print("\t");
            //Serial.print(GPS_speed);Serial.print("\t");//cm/s
            //Serial.print(GPS_ground_course);Serial.print("\t");//deg
            
            //Serial.print(_velocity_north);Serial.print("\t");
            //Serial.print(actual_speedX);Serial.print("\t");
            //Serial.print(actual_speedXf);Serial.print("\t");
            //Serial.print(vx_hat);Serial.print("\t");
            //Serial.print(_velocity_east);Serial.print("\t");
            //Serial.print(_vel_down);Serial.print("\t");
            //Serial.print(actual_speedY);Serial.print("\t");
            //Serial.print(actual_speedYf);Serial.print("\t");
            //Serial.print(vy_hat);Serial.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            //Serial.print(Control_XBf);Serial.print("\t");
            //Serial.print(Control_YBf);Serial.print("\t");
            
            //Serial.print(panError);Serial.print("\t");
            //Serial.print(tiltError);Serial.print("\t");
            //Serial.print(LError);Serial.print("\t");
            
            //Serial.print(posistion_X);Serial.print("\t");
            //Serial.print(posistion_Y);Serial.print("\t");
            //Serial.print(surface_quality);Serial.print("\t");
            
            //Serial.print(Distance_L);Serial.print("\t");
            //Serial.print(Distance_R);Serial.print("\t");
            //Serial.print(Distance_F);Serial.print("\t");
            //Serial.print(Distance_B);Serial.print("\t");
            //Serial.print(Distance_X);Serial.print("\t");
            //Serial.print(Distance_Y);Serial.print("\t");
            //Serial.print(Voltage);Serial.print("\t");
            //Serial.print(Ampere);Serial.print("\t");
            
            //Serial.print(TempMPU);Serial.print("\t");
            //Serial.print(temperaturetr);Serial.print("\t");
            //Serial.print(presser,3);Serial.print("\t");
            //Serial.print(presserf,3);Serial.print("\t");
            //Serial.print(presserfF,3);Serial.print("\t");
            //Serial.print(Altitude_baro);Serial.print("\t");
            //Serial.print(Altitude_Baro_ult);Serial.print("\t");
            Serial.print(Altitude_barof);Serial.print("\t");
            Serial.print(Altitude_sonaf);Serial.print("\t");
            Serial.print(z1_hat);Serial.print("\t"); 
            
            //Serial.print(Vz_Baro_ult);Serial.print("\t");
            //Serial.print(baro_vz);Serial.print("\t");
            //Serial.print(vz_sonaf);Serial.print("\t");
            //Serial.print(z2_hat);Serial.print("\t");
            //Serial.print(Altitude_sona);Serial.print("\t");
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial.print(vz_sona*10);Serial.print("\t");
            //Serial.print(vz_hat*10);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 

            //Serial.print(vz_hat);Serial.print("\t");
            //Serial.print(DCM10);Serial.print("\t");
            //Serial.print(DCM11);Serial.print("\t");
            //Serial.print(DCM12);Serial.print("\t");
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccXf2);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ);Serial.print("\t");
            //Serial.print(AccZf2,3);Serial.print("\t");
            //Serial.print(AccZf3,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            //Serial.print(accrZ_Earthf);Serial.print("\t");
            //Serial.print(accrZ_Earth);Serial.print("\t");
            
            Serial.print(accelRaw[XAXIS]);Serial.print("\t");
            Serial.print(accelRaw[YAXIS]);Serial.print("\t");
            Serial.print(accelRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(-GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(roll_D_rate);Serial.print("\t");
            //Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXfMO*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(_accel9250X);Serial.print("\t"); 
            //Serial.print(_accel9250Y);Serial.print("\t");
            //Serial.print(_accel9250Z);Serial.print("\t");
            
            Serial.print(ahrs_r);Serial.print("\t");
            Serial.print(ahrs_p);Serial.print("\t");  
            Serial.print(ahrs_y);Serial.print("\t");  
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
             
            //Serial.print(x_angle);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            //Serial.print(roll_I_rate);Serial.print("\t");
            
            //Serial.print(u_roll);Serial.print("\t");
            //Serial.print(u_pitch);Serial.print("\t");
            //Serial.print(u_yaw);Serial.print("\t");
            
            //Serial.print(motor_FrontL);Serial.print("\t");
            //Serial.print(motor_FrontLf);Serial.print("\t");
            //Serial.print(motor_FrontR);Serial.print("\t");
            //Serial.print(motor_FrontRf);Serial.print("\t");
            //Serial.print(motor_BackL);Serial.print("\t");
            //Serial.print(motor_BackLf);Serial.print("\t");
            //Serial.print(motor_BackR);Serial.print("\t");
            //Serial.print(motor_BackRf);Serial.print("\t");
            //Serial.print(motor_Left);Serial.print("\t");
            //Serial.print(motor_Right);Serial.print("\t");
            
            //Serial.print(GPS_numSat);Serial.print("\t");
            Serial.print(Mode);Serial.print("\t");
            //Serial.print(gyroSamples2);Serial.print("\t");
            //Serial.print(1/G_Dt);Serial.print("\t");
            //Serial.print("Hz");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
      if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            time_auto++;
            //temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
            //mpu6050_Temp_Values();
            Remote_TrimACC();//motor.h
            if(Status_LED == LOW)
             {
              Status_LED = HIGH;
              //digitalWrite(Pin_LED_G, LOW);
              }
            else
            {
            Status_LED = LOW;
            //digitalWrite(Pin_LED_G, HIGH);
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 400 Hz
  }//end while roop
}
