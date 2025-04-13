///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//這是因為遙控器保護裝置/電池電壓過低/等問題所需要閃爍LED並讓裝置可能無法順利啟動
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void){
  if (error >= 100) digitalWrite(PB4, HIGH);                                        // 當錯誤值達到 100 時，LED 長亮
  else if (error_timer < millis()){                                       // 如果當前時間大於 error_timer（即錯誤閃爍時間到期）
    error_timer = millis() + 250;                                         // 設定下次錯誤閃爍的間隔為 250 毫秒
    if(error > 0 && error_counter > error + 3) error_counter = 0;         // 如果有錯誤需要報告，並且錯誤計數器大於錯誤值+3，則重置錯誤計數器
    if (error_counter < error && error_led == 0 && error > 0){            // 如果錯誤閃爍序列還未完成且 LED 為關閉狀態
      digitalWrite(PB4, HIGH);                                                       // 打開 LED
      error_led = 1;                                                       // 設定 LED 標誌為開啟
    }
    else{                                                                  // 如果錯誤閃爍序列還未完成且 LED 為開啟狀態
      digitalWrite(PB4, LOW);                                                       // 關閉 LED
      error_counter++;                                                      // 錯誤計數器加 1，用於記錄閃爍的次數
      error_led = 0;                                                        // 設定 LED 標誌為關閉
    }
  }
}
