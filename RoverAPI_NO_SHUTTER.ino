#include "MeMegaPi.h"
#include <Wire.h>
#include <MPU6050.h>

// -------------------- MOTORI --------------------
MeMegaPiDCMotor motor1(PORT1A);  // Cingolo SINISTRO
MeMegaPiDCMotor motor2(PORT1B);  // Cingolo DESTRO
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);
MeMegaPiDCMotor motor5(PORT3A);  // Braccio
MeMegaPiDCMotor motor6(PORT3B);  // Braccio
MeMegaPiDCMotor motorHand(PORT4B);

// -------------------- SENSORI --------------------
MeUltrasonicSensor ultraSensor(PORT_7);
MePort shutterPort(PORT_8);
MPU6050 imu;

// -------------------- PARAMETRI --------------------
int speed = 100;  // Variabile globale per velocità

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(shutterPort.pin1(), INPUT);
  imu.initialize();  // AGGIUNTO: inizializza IMU
  
  Serial.println("Robot pronto! (Shutter check DISABILITATO)");
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

    // -------------------- COMANDI CINGOLI --------------------
    // CONFIGURAZIONE TUA ORIGINALE: motor1 = SINISTRO, motor2 = DESTRO
    if (cmd == "Forward") {
      motor1.run(speed);    // Sinistro avanti
      motor2.run(-speed);   // Destro avanti (invertito)
    }
    else if (cmd == "Back") {
      motor1.run(-speed);   // Sinistro indietro
      motor2.run(speed);    // Destro indietro (invertito)
    }
    else if (cmd == "Left") {
      motor1.run(-speed);   // Sinistro indietro
      motor2.run(-speed);   // Destro avanti → ruota a sinistra
    }
    else if (cmd == "Right") {
      motor1.run(speed);    // Sinistro avanti
      motor2.run(speed);    // Destro indietro → ruota a destra
    }
    else if (cmd == "Stop") {
      motor1.stop();
      motor2.stop();
    }

    // -------------------- SENSORI --------------------
    else if (cmd == "ultrasonic") {
      double distance = ultraSensor.distanceCm();
      delay(100);
      Serial.println(distance);
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
    
    // -------------------- BATTERIA (NUOVO!) --------------------
    else if (cmd == "battery") {
      // ATTENZIONE: Richiede voltage divider su pin A0
      // Se NON hai voltage divider, commenta questo blocco
      int rawValue = analogRead(A0);
      float voltage = (rawValue * 5.0 / 1023.0) * 2.0;
      Serial.println(voltage);
    }

    // -------------------- BRACCIO --------------------
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

    // -------------------- MANO --------------------
    else if (cmd == "openHand") {
      time = speed;  // Usa velocità come tempo (tua logica originale)
      motorHand.run(-100);
      delay(time);
      motorHand.stop();
    }
    else if (cmd == "closeHand") {
      time = speed;  // Usa velocità come tempo (tua logica originale)
      motorHand.run(100);
      delay(time);
      motorHand.stop();
    }

    else {
      Serial.println("Comando non riconosciuto: " + cmd);
    }
  }

  delay(50);
}