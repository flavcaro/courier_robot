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

void setup() {
 Serial.begin(9600);
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
    // CONFIGURAZIONE: motor1 = cingolo SINISTRO, motor2 = cingolo DESTRO
    if (cmd == "Forward") {
      // Compensazione: cingolo sinistro leggermente più veloce per andare dritto
      int leftSpeed = speed * 1.05;  // +5% al sinistro
      if (leftSpeed > 100) leftSpeed = 100;  // Limita a 100
      motor1.run(leftSpeed);    // Cingolo sinistro avanti (compensato)
      motor2.run(-speed);       // Cingolo destro avanti (invertito)
    } else if (cmd == "Back") {
      motor1.run(-speed);   // Cingolo sinistro indietro
      motor2.run(speed);    // Cingolo destro indietro (invertito)
    } else if (cmd == "Left") {
      motor1.run(-speed);   // Cingolo sinistro indietro
      motor2.run(-speed);   // Cingolo destro avanti → ruota a sinistra
    } else if (cmd == "Right") {
      motor1.run(speed);    // Cingolo sinistro avanti
      motor2.run(speed);    // Cingolo destro indietro → ruota a destra
    } else if (cmd == "Stop") {
      motor1.stop();
      motor2.stop();
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
          
    }else if (cmd == "battery"){
      // Leggi tensione batteria da pin analogico A0
      // Assumendo voltage divider: Vbat -> R1(10k) -> A0 -> R2(10k) -> GND
      // Vout = Vbat * R2/(R1+R2) = Vbat * 0.5
      // Arduino legge 0-1023 per 0-5V, quindi Vbat = (analogRead * 5.0 / 1023) * 2
      int rawValue = analogRead(A0);
      float voltage = (rawValue * 5.0 / 1023.0) * 2.0;  // Moltiplicatore dipende dal voltage divider
      Serial.println(voltage);
      
    }else if (cmd == "closeHand"){
      time = speed;
          motorHand.run(100);
          delay(time);
          motorHand.stop();
    }else {
      Serial.println("Comando non riconosciuto: " + cmd);
    }
  }
}
