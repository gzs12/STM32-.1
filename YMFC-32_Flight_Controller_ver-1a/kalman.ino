void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 0.22 * 0.005;//0.004*0.004*0.22*0.005
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 1* 1);//1*1
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
    //Serial.print("KalmanGain: ");
    //Serial.println(KalmanGain);
    //delay(20);
    //Serial.print("Filtered State: ");
    //Serial.println(*KalmanState);
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