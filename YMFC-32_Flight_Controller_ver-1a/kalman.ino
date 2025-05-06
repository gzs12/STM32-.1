void kalman_1d(float KalmanUncertainty) {  
  angle_pitch=angle_pitch+0.004*gyro_pitch;

  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  // 如果IMU有偏航，將滾轉角度轉移到俯仰角度(這是歐拉公式)

  angle_roll=angle_roll+0.004*gyro_roll;

  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066); 

  if (isnan(angle_pitch) || isnan(angle_roll)) { ////////
    angle_pitch = 0;/////////////////////////
    angle_roll = 0;/////////////////////
    }


  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 0.22 * 0.005;//0.004*0.004*0.22*0.005


   KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 1*1);

  angle_pitch=angle_pitch+KalmanGain * (angle_pitch_acc-angle_pitch);

  angle_roll=angle_roll+KalmanGain * (angle_roll_acc-angle_roll);

  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  
  Kalman1DOutput[1]=KalmanUncertainty;
    //Serial.print("KalmanGain: ");
    //Serial.println(KalmanGain);
    //delay(20);
    //Serial.print("KalmanUncertainty: ");
    //Serial.println(*KalmanUncertainty);
    //delay(20);
    //Serial.print("angle_roll:");
    //Serial.println(angle_roll);
    //delay(20);
    //Serial.print("angle_pitch:");
    //Serial.println(angle_pitch);
}
