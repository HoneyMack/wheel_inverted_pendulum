// 倒立振子のコントローラー

// ライブラリのインクルード
#include <Arduino.h>
#include <SAMD21turboPWM.h>
#include <Arduino_LSM6DS3.h>
#include "observer.hpp"
#include <Filters.h>
#include <Filters/Butterworth.hpp>


//制御用ピン設定
const uint8_t PWM_CONTROL_PIN = A0;//PWMデューティ比制御用入力ピン
const uint8_t L_PWM_PIN1 = 5; // PWM出力ピン
const uint8_t L_PWM_PIN2 = 6; // PWM出力ピン
const uint8_t R_PWM_PIN1 = 8; // PWM出力ピン
const uint8_t R_PWM_PIN2 = 12; // PWM出力ピン

//タクトスイッチピン
const uint8_t SWITCH_PIN = 15; // A1ピン

// PWM出力
TurboPWM pwm_out;

//バイアス・ノイズ除去
const uint8_t NUM_OF_SAMPLES = 100;
//バターワースフィルタ
const float f_s = 104; // [Hz]
const float f_c = 5; // [Hz]
const float f_n = 2 * f_c / f_s;
const float T_s = 1 / f_s;
const float T_c = 1 / f_c;
auto gyro_lowpass_filter = butter<4>(f_n);

const float alpha = T_c / (T_s + T_c); //姿勢角推定用の相補フィルタの係数
double theta_offset = 0.0;
double theta_est = 0.0;


struct Vector3 {
   float x, y, z;
};

Vector3 gyro_bias, gyro, accel;


//状態空間表現
// J_M *100倍
// float A[NUM_STATES][NUM_STATES] = {
//    {0.0000e+00,1.0000e+00,0.0000e+00},
//    {6.2025e+01,0.0000e+00,7.0813e+00},
//    {-1.5758e+01,0.0000e+00,-3.1406e+01},
// };
// float B[NUM_STATES] = { 0, -5.7338e+01, 2.5430e+02 };

// // J_M *10倍
// float A[NUM_STATES][NUM_STATES] = {
// {0.0000e+00,1.0000e+00,0.0000e+00},
// {9.0903e+01,0.0000e+00,6.4639e+01},
// {-1.4384e+02,0.0000e+00,-2.8667e+02},
// };
// float B[NUM_STATES] = { 0.0000e+00,-5.2338e+02,2.3212e+03 };

// float C[NUM_OBSERVATIONS][NUM_STATES] = {
//    {0.0000e+00,1.0000e+00,0.0000e+00},
// };

// J_M *1倍
float A[NUM_STATES][NUM_STATES] = {
   {0.0000e+00,1.0000e+00,0.0000e+00},
   {2.3172e+02,0.0000e+00,3.4529e+02},
   {-7.6835e+02,0.0000e+00,-1.5314e+03},
};
float B[NUM_STATES] = { 0.0000e+00,-2.7959e+03,1.2400e+04 };

float C[NUM_OBSERVATIONS][NUM_STATES] = {
   //{1.0000e+00,0.0000e+00,0.0000e+00},
   {0.0000e+00,1.0000e+00,0.0000e+00},
};


//オブザーバーゲイン
// float L[NUM_STATES][NUM_OBSERVATIONS] = { { 1.00379706e+00}, {-1.51138837e+03}, {6.70345312e+03} }; // 
// float L[NUM_STATES][NUM_OBSERVATIONS] = { { 1.02791956e+00}, {-1.49138837e+03}, {6.61586851e+03} };
// float L[NUM_STATES][NUM_OBSERVATIONS] = { { 4.79706009e+00}, {-1.33138837e+03}, {5.94101993e+03} };
// float L[NUM_STATES][NUM_OBSERVATIONS] = { { 186.14836699}, {168.59397819}, {-477.11901401} };
// float L[NUM_STATES][NUM_OBSERVATIONS] = { { 1.18514837}, {-11.40602181}, {67.88743963} };
// float L[NUM_STATES][NUM_OBSERVATIONS] = { //J_M *10   
//    {6.0175e+01,7.9292e+00},
//    {4.5930e+01,-1.4685e+02},
//    {-7.5476e+01,7.2360e+02},
// };

// float L[NUM_STATES][NUM_OBSERVATIONS] = { //J_M *1
//    // {6.6486e+01,2.0855e+00},
//    // {1.8497e+02,-1.3979e+03},
//    // {-6.7480e+02,6.2126e+03},

//    {2.0264e+01,1.0948e+00},
//    {2.1773e+02,-1.4917e+03},
//    {-7.3945e+02,6.6167e+03}
// };

float L[NUM_STATES][NUM_OBSERVATIONS] = { //J_M *1
   {1.1025e+00},
   {-1.4714e+03},
   {6.5291e+03},
};

//フィードバックゲイン
// float F[NUM_STATES] = { 1.1024e+01,1.4622e+00,2.5099e-01 }; // J_M* 100
// float F[NUM_STATES] = { 9.2854e+00,1.2401e+00,2.5099e-01 }; // J_M* 1
float F[NUM_STATES] = { 9.1138e+00,1.2182e+00,2.5099e-01 }; //等倍

Observer observer(A, B, C, L);

float y[NUM_OBSERVATIONS] = { 0 };//観測
float u = 0; //モータ出力
float dt = -1; //サンプリング周期
unsigned long before_time = 0, current_time = 0; //経過時間保持

//出力値の制限
const float V_M = 5.0; //モーター電源電圧[V]
const float MAX_OUTPUT = 4.5; //[V]
const float MIN_OUTPUT = 1.0; //[V] // モーターが動き出す直前の電圧
//動作モードの設定
enum Mode {
   STOP,
   INITIALIZE,
   RUN
};

Mode mode = STOP;
bool buildin_led_state = false;
int counter = 0;


void update_gyro() {
   //ジャイロセンサの値を更新
   IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
   //バイアスを除去
   gyro.x -= gyro_bias.x;
   gyro.y -= gyro_bias.y;
   gyro.z -= gyro_bias.z;

   // deg/s -> rad/s
   gyro.x *= 2 * 3.14159265 / 180.0;
   gyro.y *= 2 * 3.14159265 / 180.0;
   gyro.z *= 2 * 3.14159265 / 180.0;
}
void update_accel() {
   //加速度センサの値を更新
   IMU.readAcceleration(accel.x, accel.y, accel.z);
}

float calculate_u() {
   float voltage = 0.0;
   for (int i = 0; i < NUM_STATES; i++) {
      voltage += F[i] * observer.x_est[i];
   }
   // 出力値を制限
   voltage = (voltage > MAX_OUTPUT) ? MAX_OUTPUT : voltage;
   voltage = (voltage < -MAX_OUTPUT) ? -MAX_OUTPUT : voltage;

   // 電圧が低いと動かないので、u=0でギリギリ動く手前となるように補正
   float u_scaled = (MAX_OUTPUT - MIN_OUTPUT) / (MAX_OUTPUT)*abs(voltage) + MIN_OUTPUT;
   voltage = (voltage > 0) ? u_scaled : -u_scaled;

   return voltage;
}

void drive_motor(float duty_cycle, uint8_t PIN1, uint8_t PIN2) {
   if (duty_cycle > 0) {
      pwm_out.analogWrite(PIN1, (int)(1000 * duty_cycle));
      pwm_out.analogWrite(PIN2, 0);
   }
   else {
      pwm_out.analogWrite(PIN1, 0);
      pwm_out.analogWrite(PIN2, (int)(1000 * -duty_cycle));
   }
}

void serial_debug() {
   if (mode == Mode::RUN) {
      if (counter > 100) {
         counter = 0;
         // // シリアル出力
         // Serial.print(gyro.x);
         // Serial.print(",");
         // Serial.print(gyro.y);
         // Serial.print(",");
         // Serial.print(gyro.z);
         // Serial.print(",");
         // Serial.print(accel.x);
         // Serial.print(",");
         // Serial.print(accel.y); //鉛直上向きが正
         // Serial.print(",");
         // Serial.print(accel.z); //水平後方が正
         // Serial.println();

         //オブザーバの状態・出力uをシリアル出力
         Serial.print(observer.x_est[0]);
         Serial.print(",");
         Serial.print(observer.x_est[1]);
         Serial.print(",");
         Serial.print(observer.x_est[2]);
         Serial.print(",");
         Serial.print(u);
         Serial.print(",");
         Serial.print(y[0]);
         Serial.print(",");
         Serial.print(y[1]);
         Serial.print(",");
         Serial.print(dt);
         Serial.println();
      }
   }
   counter++;
}

void setup() {
   //シリアル通信の初期化
   Serial.begin(115200);

   //タクトスイッチの初期化
   pinMode(SWITCH_PIN, INPUT);
   //表示用LEDの初期化
   pinMode(LED_BUILTIN, OUTPUT);

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
   float sum_gyros[3] = { 0.f, 0.f, 0.f };
   for (int i = 0; i < NUM_OF_SAMPLES; i++) {
      // ジャイロセンサの値が更新されるまで待つ
      while (!IMU.gyroscopeAvailable())
         delay(1);

      // ジャイロセンサの値を更新
      update_gyro();
      sum_gyros[0] += gyro.x;
      sum_gyros[1] += gyro.y;
      sum_gyros[2] += gyro.z;
   }
   gyro_bias.x = sum_gyros[0] / NUM_OF_SAMPLES;
   gyro_bias.y = sum_gyros[1] / NUM_OF_SAMPLES;
   gyro_bias.z = sum_gyros[2] / NUM_OF_SAMPLES;
}


void mode_stop_action() {
   //停止モードのときはLEDを点滅させ、シリアル出力でSTOPと表示
   if (counter > 20) {
      digitalWrite(LED_BUILTIN, buildin_led_state);
      Serial.println("current mode:STOP");
      counter = 0;
      buildin_led_state = !buildin_led_state;
   }

   // タクトスイッチが押されたら動作モードに移行
   if (digitalRead(SWITCH_PIN) == HIGH) {
      mode = Mode::INITIALIZE;
      digitalWrite(LED_BUILTIN, LOW);//LEDを消灯
   }
   counter++;
   delay(100);
}

void mode_initialize_action() {
   // オブザーバーの初期化
   float x_est_init[NUM_STATES] = { 0, 0, 0 };
   observer.initialize(x_est_init);

   //thetaのオフセットを計算
   theta_offset = 0.0;
   for (int i = 0; i < NUM_OF_SAMPLES / 2; i++) {
      while (!IMU.accelerationAvailable())
         delay(1);
      update_accel();
      theta_offset += atan2(-accel.y, -accel.z);
   }
   theta_offset /= (NUM_OF_SAMPLES / 2);


   delay(500); //押してから少しの間動かないようにする

   //観測値の初期値を設定
   while (!IMU.gyroscopeAvailable() || !IMU.accelerationAvailable())
      delay(1);
   update_gyro();
   update_accel();
   before_time = millis();// 時間の初期化
   y[0] = gyro.x;


   // 動作モードに移行
   mode = Mode::RUN;
}

void mode_run_action() {
   // センサ値取得
   if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
      update_gyro(); update_accel();

      // 時間の更新
      current_time = millis();
      dt = (current_time - before_time) / 1000.0;
      //時間の更新
      before_time = current_time;

      //姿勢の計算
      theta_est = alpha * (theta_est + gyro.x * dt) + (1 - alpha) * (atan2(-accel.y, -accel.z) - theta_offset);

      //観測値の更新
      // 角速度のみ観測値として使う場合
      y[0] = gyro_lowpass_filter(gyro.x); // ローパスフィルタを通す

      // 姿勢も観測値として使う場合
      // y[0] = theta_est;
      // y[1] = gyro_lowpass_filter(gyro.x); // ローパスフィルタを通す

      // オブザーバの更新
      observer.step(y, u, dt);

      // 状態フィードバックの計算
      u = calculate_u();

      // PWM出力:正の値でCW回転、負の値でCCW回転
      float duty_cycle = -u / V_M;
      drive_motor(duty_cycle, R_PWM_PIN1, R_PWM_PIN2);
      drive_motor(duty_cycle, L_PWM_PIN1, L_PWM_PIN2);

      //本体が横に倒れるか、ボタンを押したら停止モードに移行
      if (abs(accel.z) > 2.0 || digitalRead(SWITCH_PIN) == HIGH) {
         mode = Mode::STOP;
         // モーターを停止
         drive_motor(0, R_PWM_PIN1, R_PWM_PIN2);
         drive_motor(0, L_PWM_PIN1, L_PWM_PIN2);
         delay(500);
      }
   }
}

void loop() {
   if (mode == Mode::STOP) {
      // 停止モード
      mode_stop_action();
   }
   else if (mode == Mode::INITIALIZE) {
      // 初期化モード
      mode_initialize_action();
   }
   else {
      // 動作モード
      mode_run_action();
   }
   serial_debug();
}
