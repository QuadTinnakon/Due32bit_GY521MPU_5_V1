/*
//An attitude and heading reference system (AHRS)  By tinnakon 26_05_2557(2014)
project_Quad 32 bit Arduino Due
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
#define Kp 0.35 //0.22 0.15  //0.2 1.62	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0005011 //0.122 0.072 0.096 0.05	// integral gain governs rate of convergence of gyroscope biases
#define Kp_mag 0.25  //0.22
#define Ki_mag 0.0005101 //0.142
float exInt , eyInt , ezInt ;	// scaled integral error
float q0, q1 , q2 , q3 ;	// quaternion elements representing the estimated orientation
float ahrs_p,ahrs_r,ahrs_y;
float Heading = 0.0;
float setHeading = 0.0;
float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;

void ahrs_initialize()
{
  exInt=0.0;
  eyInt=0.0;
  ezInt=0.0;
  q0=1.0;
  q1=0.0;
  q2=0.0;
  q3=0.0;
}
boolean isSwitched(float previousError, float currentError) {
  if ((previousError > 0.0 &&  currentError < 0.0) || (previousError < 0.0 &&  currentError > 0.0)) {
    return true;
  }
  return false;
}
void ahrs_updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt){
  float norm1,halfT;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez, ezMag;
  halfT=G_Dt/2.0;        
  norm1 = sqrt(ax*ax + ay*ay + az*az); // normalise the measurements    
  ax = ax / norm1;
  ay = ay / norm1;
  az = az / norm1;
  norm1 = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm1;
  my = my / norm1;
  mz = mz / norm1;         
  hx = mx*DCM00 + my*DCM01 + mz*DCM02;  // compute reference direction of flux, Earth Frame
  hy = mx*DCM10 + my*DCM11 + mz*DCM12;
  hz = mx*DCM20 + my*DCM21 + mz*DCM22;         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz; 
  vx = DCM20;//2*(q1q3 - q0q2)  // estimated direction of gravity and flux (v and w)
  vy = DCM21;//2*(q0q1 + q2q3)
  vz = DCM22;//q0q0 - q1q1 - q2q2 + q3q3
  wx = bx*DCM00 + bz*DCM20;
  wy = bx*DCM01 + bz*DCM21;
  wz = bx*DCM02 + bz*DCM22;  
  ex = (ay*vz - az*vy);  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
  //ez = (ax*vy - ay*vx);// + (mx*wy - my*wx)
  ezMag = (mx*wy - my*wx);
  // integral error scaled integral gain
  //exInt += ex*Ki*G_Dt;
  //eyInt += ey*Ki*G_Dt;
  //ezInt += (ez*Ki + ezMag*Ki_mag)*G_Dt;
  //*
  exInt += ex*Ki;
    if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
  eyInt += ey*Ki;
    if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;
  float ezi = (ez*Ki + ezMag*Ki_mag);
    ezInt += ezi;
    if (isSwitched(previousEz,ezi)) {
    ezInt = 0.0;
  }
  previousEz = ezi;
  gx += (Kp*ex + exInt);  // adjusted gyroscope measurements
  gy += (Kp*ey + eyInt);
  gz += (Kp*ez + ezMag*Kp_mag + ezInt);
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;  // integrate quaternion rate and normalise
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  norm1 = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  // normalise quaternion
  q0 = q0 / norm1;
  q1 = q1 / norm1;
  q2 = q2 / norm1;
  q3 = q3 / norm1;
  //float q0q0 = q0*q0;
  float q0q1 = q0*q1;// auxiliary variables to reduce number of repeated operations
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;  
  //direction cosine matrix (DCM), Rotation matrix , Rotated Frame to Stationary Frame XYZ  ,, Quaternions_and_spatial_rotation
  DCM00 = 2*(0.5 - q2q2 - q3q3);//2*(0.5 - q2q2 - q3q3);//=q0q0 + q1q1 - q2q2 - q3q3
  DCM01 = 2*(q1q2 - q0q3);//2*(q0q1 + q2q3)
  DCM02 = 2*(q1q3 + q0q2);//2*(q1q3 - q0q2); 2*(q0q2 - q1q3)
  DCM10 = 2*(q1q2 + q0q3);
  DCM11 = 2*(0.5 - q1q1 - q3q3);//2*(0.5 - q1q1 - q3q3);//q0q0 - q1q1 + q2q2 - q3q3
  DCM12 = 2*(q2q3 - q0q1);
  DCM20 = 2*(q1q3 - q0q2);//-sin pitch
  DCM21 = 2*(q2q3 + q0q1);
  DCM22 = 2*(0.5 - q1q1 - q2q2);//2*(0.5 - q1q1 - q2q2);//=q0q0 - q1q1 - q2q2 + q3q3
  //DCM23 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
  //ahrs_toEuler();
  ahrs_y=degrees(atan2(DCM10, DCM00));//2*q0*q0+2*q1*q1-1)
  ahrs_p=degrees(-asin(DCM20)); // theta
  //ahrs_p=acos(22);//http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
  ahrs_r=degrees(atan2(DCM21, DCM22)); // phi
  cos_rollcos_pitch = DCM22;
  if(cos_rollcos_pitch <= 0.866025403)//30 deg = 0.866025403
    {
      cos_rollcos_pitch = 0.866025403;
    }
////transition between 0 and 360 or -180 and 180 ,By aeroquad//////////    
     Heading = ahrs_y - setHeading;
    if (ahrs_y <= (setHeading - 180)) {
      Heading += 360;
    }
    if (ahrs_y >= (setHeading + 180)) {
      Heading -= 360;
    } 
//////Earth Frame///////////////
//accrX_Earth = hx;
//accrY_Earth = hy;
//accrZ_Earth = hz;
accrX_Earthf = (AccXf*DCM00 + AccYf*DCM01 + AccZf*DCM02)*-1.0;
accrY_Earthf = (AccXf*DCM10 + AccYf*DCM11 + AccZf*DCM12)*-1.0;
accrZ_Earthf = (AccXf*DCM20 + AccYf*DCM21 + AccZf*DCM22) - acc_offsetZ2;//acc_offsetZ2
accrX_Earth = accrX_Earth + (accrX_Earthf - accrX_Earth)*G_Dt*32.51245;
accrY_Earth = accrY_Earth + (accrY_Earthf - accrY_Earth)*G_Dt*32.51245;
accrZ_Earth = accrZ_Earth + (accrZ_Earthf - accrZ_Earth)*G_Dt*32.51245;//22.51245 42.5 18.5 Low pass filter ,smoothing factor  α := dt / (RC + dt)
applyDeadband(accrX_Earthf, 0.02);//+-0.03 m/s^2
applyDeadband(accrY_Earthf, 0.02);//+-0.03 m/s^2
applyDeadband(accrZ_Earthf, 0.02);//+-0.03 m/s^2
}
///////////////////Filter FourthOrder ///////////////////////////////////////
struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
} fourthOrder[4];
float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters)
{
  // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567
  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809
  float output;
  output = _b0 * currentInput                + 
           _b1 * filterParameters->inputTm1  + 
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;
  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;
  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;
  return output;
}
void setupFourthOrder()
{
  fourthOrder[XAXIS].inputTm1 = 0.0;
  fourthOrder[XAXIS].inputTm2 = 0.0;
  fourthOrder[XAXIS].inputTm3 = 0.0;
  fourthOrder[XAXIS].inputTm4 = 0.0;
  fourthOrder[XAXIS].outputTm1 = 0.0;
  fourthOrder[XAXIS].outputTm2 = 0.0;
  fourthOrder[XAXIS].outputTm3 = 0.0;
  fourthOrder[XAXIS].outputTm4 = 0.0;
  
  fourthOrder[YAXIS].inputTm1 = 0.0;
  fourthOrder[YAXIS].inputTm2 = 0.0;
  fourthOrder[YAXIS].inputTm3 = 0.0;
  fourthOrder[YAXIS].inputTm4 = 0.0;
  fourthOrder[YAXIS].outputTm1 = 0.0;
  fourthOrder[YAXIS].outputTm2 = 0.0;
  fourthOrder[YAXIS].outputTm3 = 0.0;
  fourthOrder[YAXIS].outputTm4 = 0.0;
  
  fourthOrder[ZAXIS].inputTm1 = acc_offsetZ;
  fourthOrder[ZAXIS].inputTm2 = acc_offsetZ;
  fourthOrder[ZAXIS].inputTm3 = acc_offsetZ;
  fourthOrder[ZAXIS].inputTm4 = acc_offsetZ;
  fourthOrder[ZAXIS].outputTm1 = acc_offsetZ;
  fourthOrder[ZAXIS].outputTm2 = acc_offsetZ;
  fourthOrder[ZAXIS].outputTm3 = acc_offsetZ;
  fourthOrder[ZAXIS].outputTm4 = acc_offsetZ;
}
