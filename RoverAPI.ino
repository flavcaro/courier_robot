#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);
MeMegaPiDCMotor motor5(PORT3A);
MeMegaPiDCMotor motor6(PORT3B);
MeMegaPiDCMotor motorHand(PORT4B);
MeUltrasonicSensor ultraSensor(PORT_7);

int speed = 100;  // di default

// Pin per lettura tensione batteria
#define BATTERY_PIN A0
// Fattore di conversione per divisore di tensione
// Se usi un divisore 2:1 (es. 10kΩ + 10kΩ), il fattore è 2.0
// Se colleghi direttamente (batteria 2S LiPo max 8.4V, Arduino max 5V), usa un divisore!
#define VOLTAGE_DIVIDER_FACTOR 2.0
// Tensione di riferimento Arduino (5V per MegaPi)
#define ARDUINO_VREF 5.0

void setup() {
 Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // es: "Forward:150"
    input.trim();  // Rimuove spazi o \r finali

    int separatore = input.indexOf(':');

    String cmd;
    int time = 0;
    if (separatore != -1) {
      cmd = input.substring(0, separatore);
      speed = input.substring(separatore + 1).toInt();
    } else {
      cmd = input;  // es: "Stop"
    }

    // Esegui comando
    if (cmd == "Forward") {
      motor1.run(speed);
      motor2.run(speed);
      motor3.run(-speed);
      motor4.run(-speed);
    } else if (cmd == "Back") {
      motor1.run(-speed);
      motor2.run(-speed);
      motor3.run(speed);
      motor4.run(speed);
    } else if (cmd == "Left") {
      motor1.run(-speed);
      motor2.run(-speed);
      motor3.run(-speed);
      motor4.run(-speed);
    } else if (cmd == "Right") {
      motor1.run(speed);
      motor2.run(speed);
      motor3.run(speed);
      motor4.run(speed);
    } else if (cmd == "Stop") {
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop();
    } else if ( cmd == "ultrasonic"){
      double distance = ultraSensor.distanceCm();
      delay(100); 
      Serial.println(distance);
    } else if (cmd == "armUP"){
          motor5.run(80);
          motor6.run(80);
          delay(250 * 5);
          motor5.stop();
          motor6.stop();
          
    }else if (cmd == "armDown"){
          motor5.run(-80);
          motor6.run(-80);
          delay(250 * 4.5);
          motor5.stop();
          motor6.stop();
    }else if (cmd == "openHand"){
      time = speed;
          motorHand.run(-100);
          delay(time);
          motorHand.stop();
          
    }else if (cmd == "closeHand"){
      time = speed;
          motorHand.run(100);
          delay(time);
          motorHand.stop();
    }else if (cmd == "getBattery"){
      // Leggi tensione batteria
      int adcValue = analogRead(BATTERY_PIN);
      // Converti ADC (0-1023) in tensione (0-5V) e applica fattore divisore
      float voltage = (adcValue / 1023.0) * ARDUINO_VREF * VOLTAGE_DIVIDER_FACTOR;
      // Invia tensione con 2 decimali
      Serial.println(voltage, 2);
    }else {
      Serial.println("Comando non riconosciuto: " + cmd);
    }
  }
}
