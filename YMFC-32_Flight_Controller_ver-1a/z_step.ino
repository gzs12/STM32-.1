void z_step(void){
  for (int i = 0; i < 100; i++) {
     gyro_signalen();
     angle_pitch_acc = -atan2((float)acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 57.296;
     angle_roll_acc = atan2((float)acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 57.296;
   float accZ=-sin(angle_pitch_acc*(3.142/180))*acc_x+cos(angle_pitch_acc*(3.142/180))*sin(angle_roll_acc*(3.142/180))* acc_y+cos(angle_pitch_acc*(3.142/180))*cos(angle_roll_acc*(3.142/180))*acc_z;   
   accZ_offset += accZ;
  delay(2);
  }
accZ_offset = accZ_offset / 100.0;
}