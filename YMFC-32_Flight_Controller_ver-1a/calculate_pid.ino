///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID計算
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void){
  //Roll calculations
  if(12.6>=battery_voltage>=10.9){                     //增益調度pid
  pid_p_gain_roll = 0.825-(12.6-battery_voltage)/10;
  pid_p_gain_pitch = 0.825-(12.6-battery_voltage)/10;
  pid_i_gain_roll = 0.029+(12.6-battery_voltage)/450;//450
  pid_i_gain_pitch = 0.029+(12.6-battery_voltage)/450;//450
  pid_d_gain_roll =11.88-(12.6-battery_voltage)/4.73;
  pid_d_gain_pitch =11.88-(12.6-battery_voltage)/4.73;
  }
 else{
 pid_p_gain_roll=0.825;
 pid_p_gain_pitch=0.825;
 pid_i_gain_roll=0.029;
 pid_i_gain_pitch=0.029;
 pid_d_gain_roll = 11.88;
 pid_d_gain_pitch = 11.88;
 }
 float min_p = 0.69, max_p = 0.84;                //自適應pid
float min_i = 0.023, max_i = 0.031;
 if ((angle_pitch <= -40 || angle_pitch >= 40) || (angle_roll  <= -40 || angle_roll  >= 40)) {
    pid_p_gain_roll  *= 0.96;
    pid_p_gain_pitch *= 0.96;
    pid_i_gain_roll  *= 1.05;
    pid_i_gain_pitch *= 1.05;
  if(pid_p_gain_roll > max_p)pid_p_gain_roll = max_p;   
  else if(pid_p_gain_roll < min_p )pid_p_gain_roll = min_p; 
  if(pid_p_gain_pitch > max_p)pid_p_gain_pitch = max_p;   
  else if(pid_p_gain_pitch < min_p )pid_p_gain_pitch = min_p;    
  if(pid_i_gain_roll > max_i)pid_i_gain_roll = max_i;   
  else if(pid_i_gain_roll < min_i )pid_i_gain_roll = min_i;   
  if(pid_i_gain_pitch > max_i)pid_i_gain_pitch = max_i;   
  else if(pid_i_gain_pitch < min_i )pid_i_gain_pitch = min_i;   
}


  angular_acceleration_estimate = (gyro_roll_input - gyro_roll_last) ;

  pid_error_temp = pid_roll_setpoint - gyro_roll_input;
  predicted_gyro = gyro_roll_input + angular_acceleration_estimate ;  // 一階預測
  predicted_error = pid_roll_setpoint - predicted_gyro;
  total_error = pid_error_temp *0.5+ predicted_error*0.5; //0.6/0.4
  pid_error_temp=total_error;

  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;            //PID基本離散化運算公式  (誤差=目標角速度（搖桿+修正）−當前角速度（gyro）rad/s)
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;    //飽和抑制                                  P=P參數*誤差
  else if(pid_i_mem_roll < -pid_max_roll )pid_i_mem_roll = -pid_max_roll;//飽和抑制                         I=I+(I參數)*誤差
                                                                                                          // D=D參數*(誤差-上一個誤差)
  pid_d_roll=pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll +  (pid_d_roll*0.167+pid_d_last_roll*0.833);   //pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_output_roll)pid_output_roll = pid_max_output_roll;  //飽和抑制
  else if(pid_output_roll < -pid_max_output_roll )pid_output_roll = -pid_max_output_roll ;  //飽和抑制
   gyro_roll_last = gyro_roll_input;
  pid_last_roll_d_error = pid_error_temp;
  pid_d_last_roll=pid_d_roll;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////(1)
  //Pitch calculations
  


  angular_acceleration_estimate = (gyro_pitch_input - gyro_pitch_last) ;

  pid_error_temp =  pid_pitch_setpoint - gyro_pitch_input ;
  predicted_gyro = gyro_pitch_input + angular_acceleration_estimate ;  // 一階預測
  predicted_error = pid_pitch_setpoint - predicted_gyro;
  total_error = pid_error_temp *0.5+ predicted_error*0.5; 
  pid_error_temp=total_error;


  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;//飽和抑制
  else if(pid_i_mem_pitch < -pid_max_pitch)pid_i_mem_pitch = -pid_max_pitch ;//飽和抑制

  pid_d_pitch=pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch +(pid_d_pitch*0.167+pid_d_last_pitch*0.833);     // pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_output_pitch)pid_output_pitch = pid_max_output_pitch;//飽和抑制
  else if(pid_output_pitch < -pid_max_output_pitch )pid_output_pitch = -pid_max_output_pitch ;//飽和抑制
   gyro_pitch_last = gyro_pitch_input;
  pid_last_pitch_d_error = pid_error_temp;
   pid_d_last_pitch=pid_d_pitch;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////(2)
  //Yaw calculations



angular_acceleration_estimate = (gyro_yaw_input - gyro_yaw_last) ;

  pid_error_temp = pid_yaw_setpoint - gyro_yaw_input;
  predicted_gyro = gyro_yaw_input + angular_acceleration_estimate ;  // 一階預測
  predicted_error = pid_yaw_setpoint - predicted_gyro;
  total_error = pid_error_temp *0.5+ predicted_error*0.5; 
  pid_error_temp=total_error;



  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;//飽和抑制
  else if(pid_i_mem_yaw < -pid_max_yaw )pid_i_mem_yaw = -pid_max_yaw ;//飽和抑制

  pid_d_yaw=pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw +(pid_d_yaw*0.167+pid_d_last_yaw*0.833); //pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_output_yaw)pid_output_yaw = pid_max_output_yaw;//飽和抑制
  else if(pid_output_yaw < -pid_max_output_yaw)pid_output_yaw = -pid_max_output_yaw ;//飽和抑制
   gyro_yaw_last = gyro_yaw_input;
  pid_last_yaw_d_error = pid_error_temp;
  pid_d_last_yaw=pid_d_yaw;
 

    
    //pid_error_temp=DesiredVelocityVertical-VelocityVerticalKalman;
    pid_error_temp=DesiredVelocityVertical-VelocityVerticalKalman;
    pid_i_mem_high+=pid_i_gain_high* pid_error_temp;
    if(pid_i_mem_high > pid_max_high)pid_i_mem_high = pid_max_high;//飽和抑制
     else if(pid_i_mem_high < -pid_max_high)pid_i_mem_high = -pid_max_high ;//飽和抑制
     pid_output_high=pid_p_gain_high * pid_error_temp + pid_i_mem_high + pid_d_gain_high * (pid_error_temp - pid_last_high_d_error);
    if(pid_output_high > pid_max_output_high)pid_output_high = pid_max_output_high;//飽和抑制
    else if(pid_output_high< -pid_max_output_high)pid_output_high = -pid_max_output_high ;//飽和抑制
     pid_last_high_d_error = pid_error_temp;
     
    //Serial.print("pid_p_gain_roll: ");
    //Serial.println(pid_p_gain_roll);
    //Serial.print("pid_i_gain_roll: ");
    //Serial.println(pid_i_gain_roll);
    //Serial.print("pid_output_roll: ");
    //Serial.println(pid_output_roll);
    //Serial.print("pid_output_yaw:");
    //Serial.println(pid_output_yaw);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////(3)
}
