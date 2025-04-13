void barometer_signals(void){
  Wire2.beginTransmission(0x76);
  Wire2.write(0xF7);
  Wire2.endTransmission();
  Wire2.requestFrom(0x76,6);
  uint32_t press_msb = Wire2.read();
  uint32_t press_lsb = Wire2.read();
  uint32_t press_xlsb = Wire2.read();
  uint32_t temp_msb = Wire2.read();
  uint32_t temp_lsb = Wire2.read();
  uint32_t temp_xlsb = Wire2.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4) <<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
  double pressure=(double)p/100;
  AltitudeBarometer=44330*(1-pow(pressure/1013.25, 1/5.255))*100;
}