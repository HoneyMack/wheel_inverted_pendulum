// 倒立振子のコントローラー

// ライブラリのインクルード
#include <Arduino.h>
#include <SAMD21turboPWM.h>
#include <Arduino_LSM6DS3.h>
#include "observer.hpp"


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

//状態空間表現
float A[NUM_STATES][NUM_STATES] = { {0, 1, 0}, {0, 0, 0}, {0, 0, 0} };
float B[NUM_STATES] = { 0, 0, 1 };
float C[NUM_OBSERVATIONS][NUM_STATES] = { {1, 0, 0} };
//オブザーバーゲイン
float L[NUM_STATES][NUM_OBSERVATIONS] = { {0}, {0}, {0} };
//フィードバックゲイン
float F[NUM_STATES] = { 0, 0, 0 };

Observer observer(A, B, C, L);

float y[NUM_OBSERVATIONS] = { 0 };//観測
float u = 0; //モータ出力
float dt = -1; //サンプリング周期
unsigned long before_time = 0, current_time = 0; //経過時間保持

//出力値の制限
const float MAX_OUTPUT = 5.0; //[V]



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

   // オブザーバーの初期化
   float x_est_init[NUM_STATES] = { 0, 0, 0 };
   observer.initialize(x_est_init);

   //観測値の初期値を設定
   while (!IMU.gyroscopeAvailable() || !IMU.accelerationAvailable());
   before_time = millis();// 時間の初期化
   IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
   IMU.readAcceleration(accel.x, accel.y, accel.z);
   y[0] = gyro.x;
}


void loop() {
   // センサ値取得
   if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
      current_time = millis();
      IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
      IMU.readAcceleration(accel.x, accel.y, accel.z);

      gyro.x -= gyro_bias.x;
      gyro.y -= gyro_bias.y;
      gyro.z -= gyro_bias.z;

      //観測値の更新
      y[0] = gyro.x;

      // 経過時間の取得
      dt = (current_time - before_time) / 1000.0;

      //時間の更新
      before_time = current_time;

      // オブザーバの更新
      observer.step(y, u, dt);
      // 状態フィードバックの計算
      u = 0.0;
      for (int i = 0; i < NUM_STATES; i++) {
         u += F[i] * observer.x_est[i];
      }
      // 出力値を制限
      if (u > MAX_OUTPUT) {
         u = MAX_OUTPUT;
      }
      else if (u < -MAX_OUTPUT) {
         u = -MAX_OUTPUT;
      }

      // PWM出力
      float duty_cycle = abs(u) / MAX_OUTPUT;
      if (u > 0) {
         pwm_out.analogWrite(R_PWM_PIN1, (int)(1000 * duty_cycle));
         pwm_out.analogWrite(L_PWM_PIN1, (int)(1000 * duty_cycle));
         pwm_out.analogWrite(R_PWM_PIN2, 0);
         pwm_out.analogWrite(L_PWM_PIN2, 0);
      }
      else {
         pwm_out.analogWrite(R_PWM_PIN1, 0);
         pwm_out.analogWrite(L_PWM_PIN1, 0);
         pwm_out.analogWrite(R_PWM_PIN2, (int)(1000 * duty_cycle));
         pwm_out.analogWrite(L_PWM_PIN2, (int)(1000 * duty_cycle));
      }

      // // シリアル出力
      // Serial.print(gyro.x);
      // Serial.print(",");
      // Serial.print(gyro.y);
      // Serial.print(",");
      // Serial.print(gyro.z);
      // Serial.print(",");
      // Serial.print(accel.x);
      // Serial.print(",");
      // Serial.print(accel.y);
      // Serial.print(",");
      // Serial.print(accel.z);
      // Serial.println();
   }
}
