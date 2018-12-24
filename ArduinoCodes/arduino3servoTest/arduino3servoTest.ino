#include <Servo.h>

Servo servoA;
Servo servoB;
Servo servoC;

int ledTemoin = 8;
float angleA = 5;
float angleB = 5;
float angleC = 3;

void setup() {
  Serial.begin(19200);
  pinMode(ledTemoin, OUTPUT); // led temoin
  digitalWrite(ledTemoin,HIGH);
  servoA.attach(9);     //servo A
  servoB.attach(11);    //servo B
  servoC.attach(10);    //servo C
  delay(1000);
  servoA.writeMicroseconds((-2160+1260)*float(angleA)/90 + 2160);
  servoB.writeMicroseconds((-1975+1130)*float(angleB)/90 + 1975);
  servoC.writeMicroseconds((-1955+1130)*float(angleC)/90 + 1955);
}

int count = 0;

void loop() {
  digitalWrite(ledTemoin , millis() / 500 % 2 ); // led temoin clignotement
  
  if(Serial.available() > 0) {
    String a = Serial.readStringUntil('\n');
    if(a == "descendreBras"){
      angleA = 5;
      angleB = 5;
      angleC = 3;
    }else{
      a.remove(0,1);
      a.remove(a.length() - 1,1); 
      angleA = getValue(a, ',', 0).toFloat();  
      angleB = getValue(a, ',', 1).toFloat();    
      angleC = getValue(a, ',', 2).toFloat();  
    }
    servoA.writeMicroseconds((-2160+1260)*float(angleA)/90 + 2160);
    servoB.writeMicroseconds((-1975+1130)*float(angleB)/90 + 1975);
    servoC.writeMicroseconds((-1955+1130)*float(angleC)/90 + 1955);
  }

}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
