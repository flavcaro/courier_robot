#include "MeMegaPi.h"
 
MeMegaPiDCMotor motor1(PORT1A);  // Lato SINISTRO
MeMegaPiDCMotor motor2(PORT1B);  // Lato SINISTRO
MeMegaPiDCMotor motor3(PORT2A);  // Lato DESTRO
MeMegaPiDCMotor motor4(PORT2B);  // Lato DESTRO
MeMegaPiDCMotor motor5(PORT3A);
MeMegaPiDCMotor motor6(PORT3B);
MeMegaPiDCMotor motorHand(PORT4B);
MeUltrasonicSensor ultraSensor(PORT_7);
MeBuzzer buzzer;  // Buzzer per segnali audio
 
int speed = 100;  // di default
 
void setup() {
 Serial.begin(115200);
}
 
void loop() {
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
 
    // ============================================================
    // ROTAZIONE DIFFERENZIALE (un cingolo avanti, uno indietro)
    // MOLTO PIÙ EFFICIENTE della rotazione normale!
    // ============================================================
   
    // RotateRight: cingolo SINISTRO avanti, DESTRO indietro
    if (cmd == "RotateRight") {
      motor1.run(speed);      // sinistro avanti
      motor2.run(speed);      // sinistro avanti
      motor3.run(speed);      // destro indietro (positivo = indietro per loro)
      motor4.run(speed);      // destro indietro
    }
   
    // RotateLeft: cingolo SINISTRO indietro, DESTRO avanti
    else if (cmd == "RotateLeft") {
      motor1.run(-speed);     // sinistro indietro
      motor2.run(-speed);     // sinistro indietro
      motor3.run(-speed);     // destro avanti (negativo = avanti per loro)
      motor4.run(-speed);     // destro avanti
    }
   
    // RotateRightComp: rotazione destra COMPENSATA per differenze di potenza
    // Formato: "RotateRightComp:speedLeft:speedRight"
    // Esempio: "RotateRightComp:80:75" → sinistro più veloce per compensare debolezza
    else if (cmd == "RotateRightComp") {
      int sep2 = input.indexOf(':', separatore + 1);
      if (sep2 != -1) {
        int speedLeft = input.substring(separatore + 1, sep2).toInt();
        int speedRight = input.substring(sep2 + 1).toInt();
       
        motor1.run(speedLeft);      // sinistro avanti (compensato)
        motor2.run(speedLeft);
        motor3.run(speedRight);     // destro indietro (compensato)
        motor4.run(speedRight);
      }
    }
   
    // RotateLeftComp: rotazione sinistra COMPENSATA
    // Formato: "RotateLeftComp:speedLeft:speedRight"
    else if (cmd == "RotateLeftComp") {
      int sep2 = input.indexOf(':', separatore + 1);
      if (sep2 != -1) {
        int speedLeft = input.substring(separatore + 1, sep2).toInt();
        int speedRight = input.substring(sep2 + 1).toInt();
       
        motor1.run(-speedLeft);     // sinistro indietro (compensato)
        motor2.run(-speedLeft);
        motor3.run(-speedRight);    // destro avanti (compensato)
        motor4.run(-speedRight);
      }
    }
   
    // ============================================================
    // COMANDI PER COMPENSAZIONE DERIVA
    // ============================================================
   
    // Comando avanti compensato: "ForwardComp:speed_left:speed_right"
    // Esempio: "ForwardComp:65:70" → sinistro 65%, destro 70%
    else if (cmd == "ForwardComp") {
      int sep2 = input.indexOf(':', separatore + 1);
      if (sep2 != -1) {
        int speedLeft = input.substring(separatore + 1, sep2).toInt();
        int speedRight = input.substring(sep2 + 1).toInt();
       
        motor1.run(speedLeft);
        motor2.run(speedLeft);
        motor3.run(-speedRight);
        motor4.run(-speedRight);
      }
    }
   
    // Comando indietro compensato
    else if (cmd == "BackComp") {
      int sep2 = input.indexOf(':', separatore + 1);
      if (sep2 != -1) {
        int speedLeft = input.substring(separatore + 1, sep2).toInt();
        int speedRight = input.substring(sep2 + 1).toInt();
       
        motor1.run(-speedLeft);
        motor2.run(-speedLeft);
        motor3.run(speedRight);
        motor4.run(speedRight);
      }
    }
   
    // ============================================================
    // COMANDI ORIGINALI (mantenuti per compatibilità)
    // ============================================================
   
    else if (cmd == "Forward") {
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
    } else if (cmd == "ultrasonic"){
      double distance = ultraSensor.distanceCm();
      delay(100);
      Serial.println(distance);
    } else if (cmd == "armUP"){
      motor5.run(80);
      motor6.run(80);
      delay(250 * 12);  // 3000ms - movimento completo SU (contro gravità serve più tempo)
      motor5.stop();
      motor6.stop();
    } else if (cmd == "armDown"){
      motor5.run(-80);
      motor6.run(-80);
      delay(250 * 6);  // 1500ms - movimento completo GIÙ (gravità aiuta, più veloce)
      motor5.stop();
      motor6.stop();
    } else if (cmd == "openHand"){
      time = speed;
      motorHand.run(-100);
      delay(time);
      motorHand.stop();
    } else if (cmd == "closeHand"){
      time = speed;
      motorHand.run(100);
      delay(time);
      motorHand.stop();
    } else if (cmd == "beep"){
      // Beep di errore: 3 bip corti
      for (int i = 0; i < 3; i++) {
        buzzer.tone(1000, 200);  // 1000Hz per 200ms
        delay(300);
      }
    } else {
      Serial.println("Comando non riconosciuto: " + cmd);
    }
  }
}
 
 