///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//這是MPU6050初始化
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void){
  Wire2.beginTransmission(gyro_address);                        //gyro_address是MPU6050的通訊地址 (開始I2C通訊)
  Wire2.write(0x6B);                                            // 指定 PWR_MGMT_1 (電源管理) 寄存器地址
  Wire2.write(0x00);                                            // 設置寄存器值為 00000000，啟動陀螺儀（退出休眠模式）
  Wire2.endTransmission();                                      // 結束 I2C 傳輸

  Wire2.beginTransmission(gyro_address);                        
  Wire2.write(0x1B);                                            // 指定 GYRO_CONFIG (陀螺儀設定) 寄存器地址
  Wire2.write(0x08);                                            // 設置寄存器值為 00001000，選擇 500°/s 的最大量程
  Wire2.endTransmission();                                     

  Wire2.beginTransmission(gyro_address);                        
  Wire2.write(0x1C);                                            // 指定 ACCEL_CONFIG (加速度計設定) 寄存器地址
  Wire2.write(0x10);                                            // 設置寄存器值為 00010000，選擇 ±8g 的最大量程
  Wire2.endTransmission();                                     

  Wire2.beginTransmission(gyro_address);                        
  Wire2.write(0x1A);                                           // 指定 CONFIG (設定) 寄存器地址
  Wire2.write(0x03);                                           // 設置寄存器值為 00000011，將 DLPF 設置為 ~43Hz，減少高頻噪音03
  Wire2.endTransmission();                                      
}
