void kalman_2d(void){
  Acc = {AccZInertial};
  S=F*S+G*Acc;
  P=F*P*~F+Q;
  L=H*P*~H+R;
  K = P * ~H * Inverse(L);
  M = {AltitudeBarometer};
  S=S+K*(M-H*S);
  AltitudeKalman=S(0,0); 
  VelocityVerticalKalman=S(1,0); 
  P=(I-K*H)*P;
}