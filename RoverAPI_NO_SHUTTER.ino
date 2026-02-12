#include "MeMegaPi.h"
#include <Wire.h>
#include <MPU6050.h>

// -------------------- MOTORI --------------------
MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);
MeMegaPiDCMotor motor5(PORT3A);
MeMegaPiDCMotor motor6(PORT3B);
MeMegaPiDCMotor motorHand(PORT4B);

// -------------------- SENSORI --------------------
MeUltrasonicSensor ultraSensor(PORT_7);
MePort shutterPort(PORT_8);
MPU6050 imu;

// -------------------- PARAMETRI --------------------
int speed = 100;             // usato per i cingoli (e anche come "time" per mano come nel tuo codice)
float trimLeft = 1.05;       // +5% al cingolo sinistro per andare più dritto (TARABILE)
const int MAX_PWM = 255;     // limite PWM tipico

int clampPwm(int v) {
  if (v >  MAX_PWM) return  MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return v;
}

// Applica trim al sinistro e inversione al destro
// NOTA: motor1 è il cingolo DESTRO, motor2 è il cingolo SINISTRO (fisicamente)
void driveTracks(int left, int right) {
  int l = (int)(left * trimLeft);
  int r = right;

  l = clampPwm(l);
  r = clampPwm(r);

  motor2.run(-l);     // cingolo sinistro → motor2 (invertito)
  motor1.run(r);      // cingolo destro → motor1
}

void stopTracks() {
  motor1.stop();
  motor2.stop();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(shutterPort.pin1(), INPUT);

  imu.initialize(); // nel tuo mancava

  Serial.println("Robot pronto! (Shutter check DISABILITATO per test)");
  Serial.print("TrimLeft iniziale: ");
  Serial.println(trimLeft, 2);
}

void loop() {
  int shutterState = shutterPort.dRead1();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int separatore = input.indexOf(':');
    String cmd;
    int value = 0;

    if (separatore != -1) {
      cmd = input.substring(0, separatore);
      value = input.substring(separatore + 1).toInt();
    } else {
      cmd = input;
    }

    // -------------------- COMANDI CINGOLI --------------------
    if (cmd == "Forward") {
      if (separatore != -1) speed = value;
      speed = clampPwm(speed);
      driveTracks(speed, speed);
    }
    else if (cmd == "Back") {
      if (separatore != -1) speed = value;
      speed = clampPwm(speed);
      driveTracks(-speed, -speed);
    }
    else if (cmd == "Left") {
      if (separatore != -1) speed = value;
      speed = clampPwm(speed);
      // gira sul posto: sinistro indietro, destro avanti (logico)
      driveTracks(-speed, speed);
    }
    else if (cmd == "Right") {
      if (separatore != -1) speed = value;
      speed = clampPwm(speed);
      // gira sul posto: sinistro avanti, destro indietro (logico)
      driveTracks(speed, -speed);
    }
    else if (cmd == "Stop") {
      stopTracks();
    }

    // -------------------- TRIM PER ANDARE DRITTO --------------------
    // Usa: Trim:105  => trimLeft = 1.05
    //      Trim:100  => trimLeft = 1.00 (nessuna compensazione)
    //      Trim:112  => trimLeft = 1.12
    else if (cmd == "Trim") {
      // limiti sensati: 50..150 => 0.50..1.50
      if (value >= 50 && value <= 150) {
        trimLeft = value / 100.0;
        Serial.print("TrimLeft aggiornato: ");
        Serial.println(trimLeft, 2);
      } else {
        Serial.println("Trim fuori range. Usa Trim:50..Trim:150 (0.50..1.50)");
      }
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
    else if (cmd == "battery") {
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
      delay((int)(250 * 4.5));
      motor5.stop();
      motor6.stop();
    }

    // -------------------- MANO --------------------
    // Mantengo la tua logica: openHand:XXX usa XXX come tempo, altrimenti usa speed come tempo
    else if (cmd == "openHand") {
      int timeMs = (separatore != -1) ? value : speed;
      if (timeMs < 0) timeMs = 0;
      motorHand.run(-100);
      delay(timeMs);
      motorHand.stop();
    }
    else if (cmd == "closeHand") {
      int timeMs = (separatore != -1) ? value : speed;
      if (timeMs < 0) timeMs = 0;
      motorHand.run(100);
      delay(timeMs);
      motorHand.stop();
    }

    else {
      Serial.println("Comando non riconosciuto: " + cmd);
    }
  }

  delay(50);
}
