///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//讀取陀螺儀/加速度/溫度數據
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  Wire2.beginTransmission(gyro_address);                      //gyro_address是MPU6050的通訊地址 
  Wire2.write(0x3B);                                          //表示從 0x3B 的記憶體開始讀取 加速度計與陀螺儀數據
  Wire2.endTransmission();                                     //開始通訊
  Wire2.requestFrom(gyro_address, 14);                         //總共有14個數據
  acc_yLSB = Wire2.read() << 8 | Wire2.read();                    //將高8位與低8位合併 (Y方向加速度)
  acc_xLSB = Wire2.read() << 8 | Wire2.read();                    //將高8位與低8位合併 (X方向加速度)
  acc_zLSB = Wire2.read() << 8 | Wire2.read();                    //將高8位與低8位合併 (Z方向加速度)
  temperature = Wire2.read() << 8 | Wire2.read();              //這是溫度數據
  gyro_roll = Wire2.read() << 8 | Wire2.read();                //將高8位與低8位合併(X 軸角速度)
  gyro_pitch = Wire2.read() << 8 | Wire2.read();               //將高8位與低8位合併(Y 軸角速度)
  gyro_yaw = Wire2.read() << 8 | Wire2.read();                 //將高8位與低8位合併(Z 軸角速度)
  gyro_pitch *= -1;                                            // 因傳感器安裝方式 需要取反
  gyro_yaw *= -1;                                              


  acc_yLSB -= manual_acc_pitch_cal_value;                         //減去校準值 補償傳感器的誤差
  acc_xLSB -= manual_acc_roll_cal_value;                          
  acc_y=(float)acc_yLSB/4096;                   //換算mpu6050加速度數據為單位(g)
  acc_x=(float)acc_xLSB/4096;
  acc_z=(float)acc_zLSB/4096;
  gyro_roll -= manual_gyro_roll_cal_value;                     //減去校準值 補償傳感器的誤差
  gyro_pitch -= manual_gyro_pitch_cal_value;                 
  gyro_yaw -= manual_gyro_yaw_cal_value;                      
}
