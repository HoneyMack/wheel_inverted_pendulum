// モーターのパラメータを推定
// ダイナミクスを推定するために電圧をモータドライバに印加し、モータに加わる電圧を計測しシリアル通信でPCに送るシステム


#include <Arduino.h>
#include <SAMD21turboPWM.h>
#include "MCP_ADC.h"

//制御用ピン設定
const uint8_t PWM_CONTROL_PIN = A0;//PWMデューティ比制御用入力ピン
const uint8_t PWM_PIN = 5; // PWM出力ピン

const float VOLTAGE_REFERENCE = 5.0; // 電圧計測用基準電圧
//ADC周り
const uint8_t ADC_CS_PIN = 10; // ADCのCSピン

//電流計測用抵抗値   
const float I_RESISTOR = 5.0;//1.2;//0.1;



TurboPWM pwm_out;
MCP3004 adc;
int counter = 0;
float motor_voltage = 0.0;
float resister_voltage = 0.0;

const int MAX_COUNTER = 100;

void setup() {
   Serial.begin(115200);

   //モータードライバへのPWM出力の設定
   pwm_out.setClockDivider(1, true);
   pwm_out.timer(0, 1, 0xFFF, false);
   pwm_out.enable(0, true); //PWM出力を有効にする

   //ADCの設定
   adc.begin(ADC_CS_PIN);
}

void loop() {
   //PWMデューティ比の読み取り
   float duty_cycle = analogRead(PWM_CONTROL_PIN) / 1024.0;

   //PWM出力
   pwm_out.analogWrite(PWM_PIN, (int)(1000 * duty_cycle));

   //電圧計測
   motor_voltage += adc.differentialRead(0) * VOLTAGE_REFERENCE / 1024.0;
   resister_voltage += adc.differentialRead(2) * VOLTAGE_REFERENCE / 1024.0;
   counter++;

   if (counter > MAX_COUNTER) {
      //平均化
      motor_voltage /= MAX_COUNTER;
      resister_voltage /= MAX_COUNTER;

      // //Duty比決定用信号をシリアル通信で送信
      Serial.print("duty_cycle: ");
      Serial.print(duty_cycle);
      Serial.print("\t");

      // モーター電圧計測
      Serial.print("motor voltage: ");
      Serial.print(motor_voltage, 4);
      Serial.print("\t");

      Serial.print("resister voltage: ");
      Serial.print(resister_voltage, 4);
      Serial.print("\t");

      // 電流用抵抗計測
      float current = resister_voltage / I_RESISTOR;
      Serial.print("current: ");
      Serial.print(current, 4);

      Serial.println("");
      counter = 0;
      motor_voltage = 0.0;
      resister_voltage = 0.0;
   }
   delay(10);
}