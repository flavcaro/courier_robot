#include "MeMegaPi.h"
#include <Wire.h>
#include <MPU6050.h>
 
// Motori
MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);
MeMegaPiDCMotor motor5(PORT3A);
MeMegaPiDCMotor motor6(PORT3B);
MeMegaPiDCMotor motorHand(PORT4B);
 
// Sensori
MeUltrasonicSensor ultraSensor(PORT_7);
MePort shutterPort(PORT_8);
MPU6050 imu;
 
int speed = 100;
 
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(shutterPort.pin1(), INPUT);
  Serial.println("Robot pronto! (Shutter check DISABILITATO per test)");
}
 
void loop() {
  int shutterState = shutterPort.dRead1();
 
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  
    input.trim();
 
    int separatore = input.indexOf(':');
    String cmd;
    int time = 0;
 
    if (separatore != -1) {
      cmd = input.substring(0, separatore);
      speed = input.substring(separatore + 1).toInt();
    } else {
      cmd = input;
    }
 
    // ⚠️ CONTROLLO ANTICOLLISIONE DISABILITATO PER TEST
    // if (shutterState == LOW && (cmd == "Forward" || cmd == "Back")) {
    //   Serial.println("Ostacolo rilevato! Movimento fermato.");
    //   motor1.stop();
    //   motor2.stop();
    //   motor3.stop();
    //   motor4.stop();
    // } else {
 
    // Comandi motori (SEMPRE ESEGUITI)
    // CONFIGURAZIONE: motor1 = cingolo SINISTRO, motor2 = cingolo DESTRO
    if (cmd == "Forward") {
      motor1.run(speed);    // Cingolo sinistro avanti
      motor2.run(speed);    // Cingolo destro avanti (INVERTITO: era -speed)
    }
    else if (cmd == "Back") {
      motor1.run(-speed);   // Cingolo sinistro indietro
      motor2.run(-speed);   // Cingolo destro indietro (INVERTITO: era speed)
    }
    else if (cmd == "Left") {
      motor1.run(-speed);   // Cingolo sinistro indietro
      motor2.run(speed);    // Cingolo destro avanti (INVERTITO: era -speed)
    }
    else if (cmd == "Right") {
      motor1.run(speed);    // Cingolo sinistro avanti
      motor2.run(-speed);   // Cingolo destro indietro (INVERTITO: era speed)
    }
    else if (cmd == "Stop") {
      motor1.stop();
      motor2.stop();
    }
    else if (cmd == "ultrasonic") {
      double distance = ultraSensor.distanceCm();
      delay(100);
      Serial.println(distance);
    }
    else if (cmd == "armUP") {
      motor5.run(80);
      motor6.run(80);
      delay(250 * 5);
      motor5.stop();
      motor6.stop();
    }
    else if (cmd == "armDown") {
      motor5.run(-80);
      motor6.run(-80);
      delay(250 * 4.5);
      motor5.stop();
      motor6.stop();
    }
    else if (cmd == "openHand") {
      time = speed;
      motorHand.run(-100);
      delay(time);
      motorHand.stop();
    }
    else if (cmd == "closeHand") {
      time = speed;
      motorHand.run(100);
      delay(time);
      motorHand.stop();
    }
    else if (cmd == "imu") {
      int16_t ax, ay, az, gx, gy, gz;
      imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
      Serial.print("Acc: ");
      Serial.print(ax); Serial.print(", ");
      Serial.print(ay); Serial.print(", ");
      Serial.println(az);
 
      Serial.print("Gyro: ");
      Serial.print(gx); Serial.print(", ");
      Serial.print(gy); Serial.print(", ");
      Serial.println(gz);
    }
    else if (cmd == "shutter") {
      Serial.print("Shutter state: ");
      Serial.println(shutterState);
    }
    else {
      Serial.println("Comando non riconosciuto: " + cmd);
    }
    // } // Fine else del controllo anticollisione
  }
 
  delay(50);
}
