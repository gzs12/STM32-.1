                                       // 這是自動矯正陀螺儀子程式 讀取2000次數據之後取平均值 並讓LED閃爍
void calibrate_gyro(void) {
  if (use_manual_calibration)cal_int = 2000;                                         //如果是手動校準 設定除以次數為2000
  else {
    cal_int = 0;                                                                      
    manual_gyro_pitch_cal_value = 0;                                                  
    manual_gyro_roll_cal_value = 0;                                                   
    manual_gyro_yaw_cal_value = 0;                                                   
  }

  if (cal_int != 2000) {
                                       // 進行多次讀取，計算陀螺儀偏移值的平均數（即進行校準）
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  // 進行 2000 次讀取以進行校準(使用FOR迴圈)
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                    // 每 25 次讀取改變 LED 狀態，表示正在進行校準
      gyro_signalen();                                              
      gyro_roll_cal += gyro_roll;                                                    //把每次的讀取陀螺儀值加在一起(共2000次)
      gyro_pitch_cal += gyro_pitch;                                                   
      gyro_yaw_cal += gyro_yaw;                                   
      delay(4);                                                                       //模擬每250HZ的頻率 
    }
    digitalWrite(PB4, HIGH);                                                                     //表示校準完成
    gyro_roll_cal /= 2000;                                                           //除以2000為平均值
    gyro_pitch_cal /= 2000;                                                          
    gyro_yaw_cal /= 2000;                                                             
    manual_gyro_pitch_cal_value = gyro_pitch_cal;                                     // 將 pitch 校準值設為計算的平均值
    manual_gyro_roll_cal_value = gyro_roll_cal;                                       // 將 roll 校準值設為計算的平均值
    manual_gyro_yaw_cal_value = gyro_yaw_cal;                                         // 將 yaw 校準值設為計算的平均值
  }

}
