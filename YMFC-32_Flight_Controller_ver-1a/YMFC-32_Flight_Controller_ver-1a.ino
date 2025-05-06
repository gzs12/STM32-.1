
#include <Wire.h> 
#include <BasicLinearAlgebra.h>
TwoWire Wire2(2, I2C_FAST_MODE);//  自訂義WIRE 並保持I2C在快速模式下工作
#define disable_JTAG() afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY)

using namespace BLA;
float AltitudeKalman;
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;
float AltitudeBarometer, AltitudeBarometerStartUp;
float AccZInertial;
float PrevErrorVelocityVertical, PrevItermVelocityVertical;
int RateCalibrationNumber;
float gyro_roll_acccal,gyro_pitch_acccal;
float pid_max_accoutput_roll=10;
float pid_max_accoutput_pitch=10;
float pid_p_accgain=0.8;
float accZ_offset = 0;
float pid_p_gain_high = 5;   // PID 參數（需根據實際調整）
float pid_i_gain_high = 0;//0.0015
float pid_d_gain_high = 0.2;//0.01
int pid_max_output_high=250;
int pid_max_high = 210;



float pid_p_gain_roll = 0.825;               //  恆滾 俯仰 偏航 PID參數 0.8/0.725/0.825
float pid_i_gain_roll = 0.029;              // 0.028   /0.026 /0.029 0.0314 0.295
float pid_d_gain_roll = 11.88 ;              // 12.1 /11.88/11.95
int pid_max_roll = 110;                       //飽和抑制250 210
int pid_max_output_roll=110 ;                 //飽和抑制250
float KalmanUncertaintyAngle=2*2;
float Kalman1DOutput[] ={0,0};
float pid_p_gain_pitch = pid_p_gain_roll;  
float pid_i_gain_pitch = pid_i_gain_roll;  
float pid_d_gain_pitch = pid_d_gain_roll;  
int pid_max_pitch = pid_max_roll;                //飽和抑制
int pid_max_output_pitch = pid_max_output_roll;        //飽和抑制
float pid_accoutput_roll,pid_accoutput_pitch;

float pid_p_gain_yaw = 1.3;               //2.0
float pid_i_gain_yaw = 0.02;              
float pid_d_gain_yaw = 0.0;               
int pid_max_yaw = 90;                    //飽和抑制
int pid_max_output_yaw=100;            //飽和抑制
boolean auto_level = true;                  //自穩模式(這裡沒有設置手動模式)

float accZ,AltitudeBarometerlast,AltitudeBarometerx;
int16_t manual_acc_pitch_cal_value = 60;       //設定陀螺參數 如use_manual_calibration是false則無義義
int16_t manual_acc_roll_cal_value = -189;


uint8_t use_manual_calibration = false;    
int16_t manual_gyro_pitch_cal_value = -63;
int16_t manual_gyro_roll_cal_value = -234;
int16_t manual_gyro_yaw_cal_value = 243;

uint8_t gyro_address = 0x68;               //MPU6050地址



uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;  //定義一些必須用到的參數
uint8_t highByte, lowByte, start;
uint8_t error, error_counter, error_led;
uint16_t dig_T1, dig_P1; 
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; 
int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_yLSB,acc_xLSB,acc_zLSB;
float acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;


int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float pid_accerror_temp_pitch,pid_i_accmem_roll,pid_last_accroll_d_error, pid_i_accmem_pitch,pid_last_accpitch_d_error,pid_accerror_temp_roll;
uint32_t loop_timer, error_timer;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp,gyro_roll_last;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float pid_i_mem_high,pid_output_high,DesiredVelocityVertical,VelocityVerticalKalman,pid_last_high_d_error;
float battery_voltage;
float  angle_pitch_accsin,angle_roll_accsin,angle_pitch_acctan,angle_roll_acctan;
float angular_acceleration_estimate, predicted_gyro,predicted_error,total_error,gyro_pitch_last,gyro_yaw_last;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire2.begin();         // 初始化 I2C
  Wire2.setClock(400000); // 設置 I2C 速度為 400kHz (快速模式)
  pinMode(4, INPUT_ANALOG);                                    
  F = {1, 0.004, 0, 1};  
  G = {0.5*0.004*0.004,0.004};
  H = {1, 0};
  I = {1, 0,0, 1};
  Q = G * (~G * 100.0f);  // 或者 Q = G * ~G * 100.0f;
  R = {30*30};
  P = {0, 0, 0, 0};
  S = {0,0};
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                    //讓PB3 PB4為輸出入模式
  
  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.
 
  
  pinMode(PA8, OUTPUT);   //前置燈
  digitalWrite(PA8,LOW);
  pinMode(PA9, OUTPUT);  //後置燈
  digitalWrite(PA9,LOW);
  pinMode(PB12, OUTPUT);  //前置高功率雷射
  digitalWrite(PB12,LOW);

  digitalWrite(PB3, LOW);                                                //Set output PB3 low.
  digitalWrite(PB4, HIGH);                                                //Set output PB4 high.

  Serial.begin(57600);                                        //DEBUG用
  delay(250);                                                 

  timer_setup();                                                //初始化TIMER
  delay(50);                                                 
  //Wire2.begin();                                                //初始
  Wire2.beginTransmission(gyro_address);                        //MPU6050 的位址
  error = Wire2.endTransmission();                              
  while (error != 0) {                                          
    error = 2;                                                  //endTransmission();正常輸出的話會回傳ERROR=0
    error_signal();                                             //
    delay(4);
  }

  gyro_setup();                                                 //初始化陀螺儀 加速度計

  bmp280_step();

  if (!use_manual_calibration) {
    // 在自動校準之前創建 5 秒延遲
    for (count_var = 0; count_var < 1250; count_var++) {        // 執行 1250 次迴圈，每次延遲 4 微秒，總共 5 秒
      if (count_var % 125 == 0) {                               
        digitalWrite(PB4, !digitalRead(PB4));                   
      }
      delay(4);                                                 
    }
    count_var = 0;                                            // 將計數變數重置為 0，為下一次校準做準備
  }

  calibrate_gyro();                                             //自動或手動校準子程式碼
  z_step();
  while (channel_1 < 1200 || channel_2 < 1200 || channel_3 < 980 || channel_4 < 1200)  {
    error = 3;                                                                               //這是不太合理的遙控器數據 所以產生錯誤信號
    error_signal();                                            
    delay(4);
  }
  error = 0;                                                    
  
  while (channel_3 < 980 || channel_3 > 1060)  {
    error = 4;                                                  
    error_signal();                                              //這是不太合理的遙控器數據 所以產生錯誤信號
    delay(4);
  }
  error = 0;                                                    

  
  digitalWrite(PB4, LOW);                                                 

  
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead STM32內部的值從0-4095對應0-3.3V 所以STM32內部讀取到的值是VIN/4096*3.3
  //電阻比例是1:11
  //所以STM32內部讀到的數值還要乘以11=11*(VIN/4096*3.3)=VIN/112.81
  //
  battery_voltage = (float)analogRead(4) / 112.81;  //因為電阻11:1
  while (micros() - loop_timer < 4000);                          // 確保固定 250Hz 迴圈頻率 (1/250s = 4000us)
  loop_timer = micros();                                        //從開始到現在經過的時間(微秒)
  digitalWrite(PB3, HIGH);                                                //讓綠色LED亮
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//主要程式碼
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
 VelocityVerticalKalman=0;
  error_signal();                                                                  //檢查錯誤信號
  gyro_signalen();   
                                                                //讀取加速度計 陀螺儀數據
  
  
  
  if (channel_5 >=995 && channel_5 <=1005) {      //當channel_5的值剛好等於1500則強制停下無人機
    start=0;
   }
   if ( channel_5 >= 1995  &&  channel_5 <= 2005) {      
    digitalWrite(PB12,HIGH);
   }
   else{
    digitalWrite(PB12,LOW);}
  if(channel_6>=1995 && channel_6<=2005){           
    digitalWrite(PA8,HIGH);                         //當channel_6剛好等於2000則開啟前燈
  }
  else{
    digitalWrite(PA8,LOW);}
  if(channel_6>=995 && channel_6<=1010){           //當channel_6剛好等於1000則開啟尾燈
    digitalWrite(PA9,HIGH);
  }
   else{
    digitalWrite(PA9,LOW);
   }

   


  //65.5 = 1 deg/sec 
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   // 陀螺儀輸出值是度/秒(除以65.5才是)，用來更新滾轉（roll）角速度 這是一段權重比公式 其中陀螺儀的值占比比較多(給PID的輸入參數用) 角速度 deg/s = 原始數據 / 65.5
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      


    //1:MPU6050 設定 ±500°/s 角速度 deg/s = 原始數據 / 65.5  2: 每 1 LSB = (1 / 65.5)deg/s(mpu6050輸出的是(x)lsb) 3:每次迴圈間隔 dt = 0.004 秒 4:每 LSB → 度 = (1 / 65.5 deg/s) × 0.004 s ≈ 0.0000611 度
   angle_pitch += (float)gyro_pitch * 0.0000611;                                    // 計算俯仰角度並累加到 angle_pitch 變數 
   angle_roll += (float)gyro_roll * 0.0000611;  
    


  // 0.000001066 = 0.0000611 * (3.142(PI) / 180度) ，因為 Arduino 的 sin 函數以弧度為單位
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  // 如果IMU有偏航，將滾轉角度轉移到俯仰角度(這是歐拉公式)
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  // 如果IMU有偏航，將俯仰角度轉移到滾轉角度(這是歐拉公式)
  if (isnan(angle_pitch) || isnan(angle_roll)) { ////////
    angle_pitch = 0;/////////////////////////
    angle_roll = 0;/////////////////////
    }

     //gyro_pitch=(float)gyro_pitch*1/65.5;
     //gyro_roll=(float)gyro_roll*1/65.5;

  //Serial.print("angle_pitch: ");
 // Serial.println(angle_pitch);
  //Serial.print("angle_roll: ");
 // Serial.println(angle_roll);

 // 加速度計角度計算
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    // 計算加速度計的總向量

  if (abs(acc_y) < acc_total_vector) {                                            
    angle_pitch_accsin = asin((float)acc_y / acc_total_vector) * 57.296;              // 計算俯仰角度 
  }
  if (abs(acc_x) < acc_total_vector) {                                            
    angle_roll_accsin = asin((float)acc_x / acc_total_vector) * 57.296;               // 計算滾轉角度
  }

  
                                           
   angle_pitch_acctan = -atan2((float)acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 57.296;
   angle_roll_acctan = atan2((float)acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 57.296;
   
    float angle_pitch_acc = 0.7 * angle_pitch_accsin + 0.3 * angle_pitch_acctan;  // 比較信任 asin 的穩定性
    float angle_roll_acc  = 0.7 * angle_roll_accsin  + 0.3 * angle_roll_acctan;


   
     AccZInertial=-sin(angle_pitch_acc*(3.142/180))*acc_x+cos(angle_pitch_acc*(3.142/180))*sin(angle_roll_acc*(3.142/180))* acc_y+cos(angle_pitch_acc*(3.142/180))*cos(angle_roll_acc*(3.142/180))*acc_z;   
    AccZInertial=(AccZInertial-accZ_offset)*9.81*100;
       //Serial.print("AccZInertial: ");
       //Serial.println(AccZInertial);

    

     barometer_signals();

    AltitudeBarometer-=AltitudeBarometerStartUp;
     //Serial.print("AltitudeBarometer: ");
     //Serial.println(AltitudeBarometer);    

    kalman_2d();
    //Serial.print("Altitude [cm]: ");
    //Serial.print(AltitudeKalman);
    //Serial.print(" Vertical velocity [cm/s]: ");
    //Serial.println(VelocityVerticalKalman);

   //kalman_1d(KalmanUncertaintyAngle);
   //KalmanUncertaintyAngle=Kalman1DOutput[1];

   

  //Serial.print(" KalmanAngleRoll: ");
  //Serial.println( KalmanAngleRoll);
  //Serial.print("  KalmanAnglePitch: ");
  //Serial.println( KalmanAnglePitch);

 
  
  angle_pitch= angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                    // 使用加速度計的俯仰角度修正陀螺儀俯仰角度(互補律波)
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                       // 使用加速度計的滾轉角度修正陀螺儀滾轉角度
  
    



                                          
                                             
  pitch_level_adjust = angle_pitch* 15;      // 計算俯仰角修正(可以自己改係數)15                                      
  roll_level_adjust = angle_roll *15 ;       // 計算滾轉角修正(可以自己改係數)15                   
    //Serial.print("angle_pitch: ");
    //Serial.println(angle_pitch);
    //delay(20);
    //Serial.print("pitch_level_adjust: ");
    //Serial.println(pitch_level_adjust);
    //delay(20);

  if (!auto_level) {                                                             // 如果啟用手動模式，將修正角度設為 0
    pitch_level_adjust = 0;                                                        
    roll_level_adjust = 0;                                                         
  }

 
  
  if (channel_3 < 1010 && channel_4 < 1300){
  start = 1;                                       //第一步:當遙控器油門往左下推時 START=1
   }

  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                       //第二步:當遙控器油門往回中下時 START=2 (飛機可以啟動)
    start = 2;
   

    digitalWrite(PB3, LOW);                                                                //關閉綠LED

    angle_pitch = angle_pitch_acc;                                                 // 當四軸飛行器啟動時，將加速度計的俯仰角度設置為陀螺儀的初始俯仰角度
    angle_roll = angle_roll_acc;                                                   // 當四軸飛行器啟動時，將加速度計的滾轉角度設置為陀螺儀的初始滾轉角度

    
    pid_i_mem_roll = 0;                                           //把PID計算後的某些參數先設置為0
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  
  if (start == 2 && channel_3 < 1055 && channel_4 > 1755) {
    start = 0; 
    digitalWrite(PB3, HIGH);                                                                //停止

  }

  
   DesiredVelocityVertical=0.2*(channel_3-1500);//0.2
  




pid_accoutput_pitch=0;
pid_accoutput_roll=0;



  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;//如果油門剛好在中間(1500微秒輸出)則誤差等於0
  //We need a little dead band of 16us for better results.
  if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;                 // 如果 channel_1 輸入大於 1508，設置為輸入與 1508 之間的差值 
  else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;            // 如果 channel_2 輸入小於 1492，設置為輸入與 1492 之間的差值 (是負的)

  pid_roll_setpoint -= roll_level_adjust;                                      // 從標準化的遙控俯仰輸入值中減去角度修正值
  pid_roll_setpoint /= 3.0;                                                       // 將 PID 俯仰控制器的目標值除以 3 以轉換為度數 3.0


  // PID 控制器的目標值 (每秒度數) 是由遙控器的偏航 (yaw) 輸入決定的。
  // 在此情況下，將輸入值除以3後，最大偏航速率約為 164 度每秒 ((500-8)/3 = 164d/s)。
  pid_pitch_setpoint = 0;
  
  if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                       
  pid_pitch_setpoint /= 3.0;      //3.0                                                 

  pid_yaw_setpoint = 0;
  
  if (channel_3 > 1050) {                  // 在關閉電動機時不進行偏航操作
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;           // 如果 channel_4 輸入大於 1508，設置為與 1508 之間的差值除以 3
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;        // 如果 channel_4 輸入小於 1492，設置為與 1492 之間的差值除以 3(是負的)
  }

  calculate_pid();                                               // 計算 PID 輸出   

    
if(channel_1 < 1508&&channel_1 > 1492&&channel_2 < 1508&&channel_2 > 1492){
  acc_pid();
  
}



// 需要電池電壓來進行補償。
// 使用補償濾波器來減少噪聲。
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);
    
    
 // 如果電池電壓過低（例如低於 11.0V），則開啟 LED。
  if (battery_voltage <= 10.5 && error == 0){
    error = 1;
     digitalWrite(PA8,HIGH);                         //當channel_6剛好等於2000則開啟前燈
  }
  else{
    digitalWrite(PA8,LOW);
}
  throttle = channel_3;                                                          //當作基準信號 (這是油門信號)channel_3
  if(channel_3 >= 1480 && channel_3 <= 1520){

    throttle = 1450+pid_output_high/1.2;
    }


  if (start == 2) {                                                                //如果準備好
    if (throttle > 1650) throttle = 1650;         

    

    esc_1 = 1.024*(throttle  + pid_output_pitch - pid_output_roll + pid_output_yaw-pid_accoutput_roll+pid_accoutput_pitch);         esc 1 (front-right - CCW).      這是ESC輸出的計算通用標準型式 =總-前後(往前是正往後是負)+橫滾(往右負往左正)-偏航(右旋負左旋正)
    esc_2 = 1.024*(throttle  - pid_output_pitch - pid_output_roll - pid_output_yaw-pid_accoutput_roll-pid_accoutput_pitch);         esc 2 (rear-right - CW).                                    =總+前後(往前是正往後是負)+橫滾(往右負往左正)+偏航(右旋正左旋負)
    esc_3 = 1.024*(throttle  - pid_output_pitch + pid_output_roll + pid_output_yaw+pid_accoutput_roll-pid_accoutput_pitch);         esc 3 (rear-left - CCW).                                    =總+前後(往前是正往後是負)-橫滾(往右負往左正)-偏航(右旋正左旋負)
    esc_4 = 1.024*(throttle  + pid_output_pitch + pid_output_roll - pid_output_yaw+pid_accoutput_roll+pid_accoutput_pitch);         esc 4 (front-left - CW).                                    =總-前後(往前是正往後是負)-橫滾(往右負往左正)+偏航(右旋正左旋負)
    

    if (esc_1 < 1120) esc_1 = 1100;                                               //保持馬達怠速1100
    if (esc_2 < 1120) esc_2 = 1100;                                               
    if (esc_3 < 1120) esc_3 = 1100;                                                
    if (esc_4 < 1120) esc_4 = 1100;                                              

    if (esc_1 > 2000)esc_1 = 2000;                                                 //禁止超過2000微秒
    if (esc_2 > 2000)esc_2 = 2000;                                                
    if (esc_3 > 2000)esc_3 = 2000;                                                
    if (esc_4 > 2000)esc_4 = 2000;                                                
  }

  else {
    esc_1 = 1000;                                                                   //如果START不為2則馬達不能運轉
    esc_2 = 1000;                                                                 
    esc_3 = 1000;                                                                  
    esc_4 = 1000;                                                                  
  }
  

  TIMER4_BASE->CCR1 = esc_1;                                                       //把運算出的控制脈衝寬度傳給CCR暫存器(計數每超過CCR暫存器的值就反轉輸出)
  TIMER4_BASE->CCR2 = esc_2;                                                       
  TIMER4_BASE->CCR3 = esc_3;                                                       
  TIMER4_BASE->CCR4 = esc_4;                                                       
  TIMER4_BASE->CNT = 5000;                                                        

 

 

  if (micros() - loop_timer > 4100){
    error = 5;                                      //如果超過4000微秒則出錯
   
}
  while (micros() - loop_timer < 4000);                                            // 等待直到 4000 微秒過去
  loop_timer = micros();                                                           //重置計時器以利於下一輪循環
}
