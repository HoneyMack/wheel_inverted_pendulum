// 倒立振子のコントローラー

// ライブラリのインクルード
#include <Arduino.h>
#include <SAMD21turboPWM.h>
#include <Arduino_LSM6DS3.h>


//制御用ピン設定
const uint8_t PWM_CONTROL_PIN = A0;//PWMデューティ比制御用入力ピン
const uint8_t L_PWM_PIN1 = 5; // PWM出力ピン
const uint8_t L_PWM_PIN2 = 6; // PWM出力ピン
const uint8_t R_PWM_PIN1 = 8; // PWM出力ピン
const uint8_t R_PWM_PIN2 = 12; // PWM出力ピン

//タクトスイッチピン
const uint8_t SWITCH_PIN = 2;

// PWM出力
TurboPWM pwm_out;

//バイアス除去
const uint8_t NUM_OF_SAMPLES = 100;
struct Vector3 {
   float x, y, z;
};

Vector3 gyro_bias, gyro, accel;

void setup() {
   Serial.begin(115200);

   //モータードライバへのPWM出力の設定
   pwm_out.setClockDivider(1, true);
   pwm_out.timer(0, 1, 0xFFF, false);
   pwm_out.enable(0, true); //PWM出力を有効にする

   // LSM6DS3の初期化
   if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
   }
   Serial.print("Gyroscope sample rate = ");
   Serial.print(IMU.gyroscopeSampleRate());
   Serial.println(" Hz");
   Serial.println();
   Serial.println("Gyroscope in degrees/second");

   // バイアス計算
   gyro_bias.x = gyro_bias.y = gyro_bias.z = 0.f;
   for (int i = 0; i < NUM_OF_SAMPLES; i++) {
      if (IMU.gyroscopeAvailable()) {
         IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
         gyro_bias.x += gyro.x;
         gyro_bias.y += gyro.y;
         gyro_bias.z += gyro.z;
      }
      delay(2);
   }
   gyro_bias.x /= NUM_OF_SAMPLES;
   gyro_bias.y /= NUM_OF_SAMPLES;
   gyro_bias.z /= NUM_OF_SAMPLES;
}


void loop() {

}
