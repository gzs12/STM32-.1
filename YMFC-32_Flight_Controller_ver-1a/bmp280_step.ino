 void bmp280_step(void){
 Wire2.beginTransmission(0x76); 
  Wire2.write(0xF4);
  Wire2.write(0x57);
  Wire2.endTransmission();   
  Wire2.beginTransmission(0x76);
  Wire2.write(0xF5); 
  Wire2.write(0x14);
  Wire2.endTransmission();   
  uint8_t data[24], i=0;
  Wire2.beginTransmission(0x76);
  Wire2.write(0x88);
  Wire2.endTransmission();
  Wire2.requestFrom(0x76,24);
  while(Wire2.available()){
  data[i] = Wire2.read();
  i++;
}
  dig_T1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22]; delay(250);
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    barometer_signals();
    AltitudeBarometerStartUp+=AltitudeBarometer; 
    delay(1); 
  }
  AltitudeBarometerStartUp/=2000;
  }