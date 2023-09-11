// 2つのモータードライバをPWM制御&センサの値を取得するテスト
#include <Arduino.h>
#include <SAMD21turboPWM.h>
#include <Arduino_LSM6DS3.h>

//制御用ピン設定
const uint8_t PWM_CONTROL_PIN = A0;//PWMデューティ比制御用入力ピン
const uint8_t L_PWM_PIN1 = 5; // PWM出力ピン
const uint8_t L_PWM_PIN2 = 6; // PWM出力ピン
const uint8_t R_PWM_PIN1 = 8; // PWM出力ピン
const uint8_t R_PWM_PIN2 = 12; // PWM出力ピン


const float VOLTAGE_REFERENCE = 5.0; // 電圧計測用基準電圧


TurboPWM pwm_out;

int counter = 0;
const int MAX_COUNTER = 20;

bool is_cw = true;

//センサ値取得
float gyro_x, gyro_y, gyro_z;


void setup() {
   Serial.begin(115200);

   //モータードライバへのPWM出力の設定
   pwm_out.setClockDivider(1, true);
   pwm_out.timer(0, 1, 0xFFFF, false);
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
}

void loop() {
   //PWMデューティ比の読み取り
   //float duty_cycle = analogRead(PWM_CONTROL_PIN) / 1024.0;

   float duty_cycle = 0.9;
   if (is_cw) {
      Serial.print("CW");
      //PWM出力
      pwm_out.analogWrite(R_PWM_PIN1, (int)(1000 * duty_cycle));
      pwm_out.analogWrite(L_PWM_PIN1, (int)(1000 * duty_cycle));
      pwm_out.analogWrite(R_PWM_PIN2, 0);
      pwm_out.analogWrite(L_PWM_PIN2, 0);
   }
   else {
      Serial.print("CCW");
      //PWM出力
      pwm_out.analogWrite(R_PWM_PIN1, 0);
      pwm_out.analogWrite(L_PWM_PIN1, 0);
      pwm_out.analogWrite(R_PWM_PIN2, (int)(1000 * duty_cycle));
      pwm_out.analogWrite(L_PWM_PIN2, (int)(1000 * duty_cycle));
   }

   counter++;
   if (counter > MAX_COUNTER) {
      is_cw = !is_cw;
      counter = 0;
      pwm_out.analogWrite(R_PWM_PIN1, 0);
      pwm_out.analogWrite(L_PWM_PIN1, 0);
      pwm_out.analogWrite(R_PWM_PIN2, 0);
      pwm_out.analogWrite(L_PWM_PIN2, 0);
      delay(1000);
   }

   if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
      Serial.print(" x: ");
      Serial.print(gyro_x);
      Serial.print(" y: ");
      Serial.print(gyro_y);
      Serial.print(" z: ");
      Serial.println(gyro_z);
   }
   delay(100);

}