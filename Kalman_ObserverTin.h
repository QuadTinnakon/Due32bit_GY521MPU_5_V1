/*
project_Quad 32 bit Arduino Due
//GPS
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

//A=[1 0.01;0 1]; //B=[0;0.01];  //C = [1 0];
//Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
//Q = 0.1; % A number greater than zero
//R = 0.135; % A number greater than zero
//[kalmf,L,P,M,Z] = kalman(Plant,Q,R); //%kalmf = kalmf(1,:); //M,   % innovation gain
*/
void Observer_kalman_filter()
{
///Complimentary Filter, Observer velocity vx vy vz kalman//accelerometer and GPS/////////////////////////////////////////////////////
     //GPS Speed Low Pass Filter
      //GPS_LAT1Lf = GPS_LAT1Lf + (GPS_LAT1lead - GPS_LAT1Lf)*8.2521*G_Dt;//12.5412
      //GPS_LON1Lf = GPS_LON1Lf + (GPS_LON1lead - GPS_LON1Lf)*8.2521*G_Dt;
     //actual_speedXf = actual_speedXf + (actual_speedX - actual_speedXf)*10.2*G_Dt;//5.2 10.4  //cm/s  +-400 cm/s
     //actual_speedYf = actual_speedYf + (actual_speedY - actual_speedYf)*10.2*G_Dt;//5.2 10.17
      float temp_vx = accrX_Earth*100.1 + (_velocity_north - vx_hat2)*9.2;//2.15  1.5 6.5      vx_hat = vx_hat + temp_vx*G_Dt;
      vx_hat2 = vx_hat2 + temp_vx*G_Dt;
      vx_hat = constrain(vx_hat2, -400, 400);//+-400 cm/s
      applyDeadband(vx_hat, 4.5);//4.5 10.5
      float temp_vy = accrY_Earth*100.1 + (_velocity_east - vy_hat2)*9.2;//2.15  1.5 6.5
      vy_hat2 = vy_hat2 + temp_vy*G_Dt;
      vy_hat = constrain(vy_hat2, -400, 400);
      applyDeadband(vy_hat, 4.5);//4.5 10.5
//Predicted (a priori) state estimate  Altitude
//u_z = (motor_FrontLf + motor_FrontRf + motor_BackLf + motor_BackRf);//uz - g ,,unit N *0.001691) - 9.81
u_z = accrZ_Earth;//applyDeadband = 0.1 m/s^2  = ,,(u - c*z2_hat)/m = Accz
z2_hat2 = z2_hat + u_z*G_Dt;//z2_hat = velocity ,, m/s
z1_hat2 = z1_hat + z2_hat2*G_Dt;//z1_hat = Altitude ,, m
//z1_hat = constrain(z1_hat, 0, 100);//0 - 100 m
///////////////////////////////////////////////////////////////////
//Updated (a posteriori) state estimate
//Update estimate with measurement zk
z2_hat = z2_hat2 + 0.012502*(Altitude_Baro_ult - z1_hat2) + 0.01641023*(Vz_Baro_ult - z2_hat2);//K2 = 0.0002141023, 0.0001641, 0.00145, 0.00045,,,k2 0.097502,  0.065502, 0.009
z1_hat = z1_hat2 + 0.015102*(Altitude_Baro_ult - z1_hat2) + 0.01741023*(Vz_Baro_ult - z2_hat2);//K1 =0.035102, 0.015102 0.065 0.2887  0.09187 0.01187 0.0140
}
