void acc_pid(void){
 pid_accerror_temp_roll = manual_acc_roll_cal_value - acc_xLSB;

 pid_i_accmem_roll += pid_i_gain_roll * pid_accerror_temp_roll;            //PID基本離散化運算公式  (誤差=目標角速度（搖桿+修正）−當前角速度（gyro）rad/s)
  if(pid_i_accmem_roll > pid_max_roll)pid_i_accmem_roll = pid_max_roll;    //飽和抑制                                  P=P參數*誤差
  else if(pid_i_accmem_roll < -pid_max_roll )pid_i_accmem_roll = -pid_max_roll;//飽和抑制                         I=I+(I參數)*誤差
                                                                                            // D=D參數*(誤差-上一個誤差)
  pid_accoutput_roll = pid_p_accgain * pid_accerror_temp_roll ;//+ pid_i_accmem_roll + pid_d_gain_roll * (pid_accerror_temp_roll - pid_last_accroll_d_error);
  if(pid_accoutput_roll > pid_max_accoutput_roll)pid_accoutput_roll = pid_max_accoutput_roll;  //飽和抑制
  else if(pid_accoutput_roll < -pid_max_accoutput_roll )pid_accoutput_roll = -pid_max_accoutput_roll ;  //飽和抑制
   pid_last_accroll_d_error = pid_accerror_temp_roll;


 pid_accerror_temp_pitch = manual_acc_pitch_cal_value - acc_yLSB;

 pid_i_accmem_pitch += pid_i_gain_pitch * pid_accerror_temp_pitch;            //PID基本離散化運算公式  (誤差=目標角速度（搖桿+修正）−當前角速度（gyro）rad/s)
  if(pid_i_accmem_pitch > pid_max_pitch)pid_i_accmem_pitch = pid_max_pitch;    //飽和抑制                                  P=P參數*誤差
  else if(pid_i_accmem_pitch < -pid_max_pitch )pid_i_accmem_pitch = -pid_max_pitch;//飽和抑制                         I=I+(I參數)*誤差
                                                                                            // D=D參數*(誤差-上一個誤差)
  pid_accoutput_pitch = pid_p_accgain * pid_accerror_temp_pitch; //+ pid_i_accmem_pitch + pid_d_gain_pitch * (pid_accerror_temp_pitch - pid_last_accpitch_d_error);
  if(pid_accoutput_pitch > pid_max_accoutput_pitch)pid_accoutput_pitch = pid_max_accoutput_pitch;  //飽和抑制
  else if(pid_accoutput_pitch < -pid_max_accoutput_pitch )pid_accoutput_pitch = -pid_max_accoutput_pitch ;  //飽和抑制
   pid_last_accpitch_d_error = pid_accerror_temp_pitch;
     //Serial.print("pid_accoutput_roll: ");
     //Serial.println(pid_accoutput_roll);
     // Serial.print("pid_accoutput_pitch: ");
      //Serial.println(pid_accoutput_pitch);
}