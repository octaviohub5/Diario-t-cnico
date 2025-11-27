const int echoe = 35;
const int trige = 32;
const int echoi = 26;
const int trigi = 27;
const int echod = 21;
const int trigd = 23;
const int pin2 = 2;
const int pin4 = 4;
const int pin18 = 18;
const int pin19 = 19;
const int pinEN1 = 12;
const int pinEN2 = 13;
void setup() {
  Serial.begin(115200);
  pinMode(pin2, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(pin18, OUTPUT);
  pinMode(pin19, OUTPUT);
  pinMode(pinEN1, OUTPUT);
  pinMode(pinEN2, OUTPUT);
  pinMode(echoe, INPUT);
  pinMode(trige, OUTPUT);
  pinMode(echoi, INPUT);
  pinMode(trigi, OUTPUT);
  pinMode(echod, INPUT);
  pinMode(trigd, OUTPUT);
  digitalWrite(trige, LOW); 
  digitalWrite(trigi, LOW); 
  digitalWrite(trigd, LOW); 
}

void loop() {
  //Sensor Enfrente
    digitalWrite(trige, LOW); 
    delayMicroseconds(2);
    digitalWrite(trige, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trige, LOW);

    long tiempo1 = pulseIn(echoe, HIGH);
    float distancia1 = (tiempo1 * 0.0343) / 2;

  //Sensor Izquierda
    digitalWrite(trigi, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigi, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigi, LOW);

    long tiempo2 = pulseIn(echoi, HIGH);
    float distancia2 = (tiempo2 * 0.0343) / 2;

  //Sensor Derecha
    digitalWrite(trigd, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigd, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigd, LOW);

    long tiempo3 = pulseIn(echod, HIGH);
    float distancia3 = (tiempo3 * 0.0343) / 2;

  analogWrite(pinEN1, 100);
  analogWrite(pinEN2, 100);

  if (distancia1 < 20 && distancia2 < distancia3){
   digitalWrite(pin2,LOW);  digitalWrite(pin18,LOW);
   digitalWrite(pin4,HIGH); digitalWrite(pin19,HIGH);
   Serial.println("Atras");
   delay(1000);
   digitalWrite(pin2,HIGH);  digitalWrite(pin18,HIGH);
   digitalWrite(pin4,LOW); digitalWrite(pin19,LOW);
   Serial.println("Derecha");
  }
  else if (distancia1 < 20 && distancia2 > distancia3){
    digitalWrite(pin2,HIGH); digitalWrite(pin18,LOW);
    digitalWrite(pin4,LOW);  digitalWrite(pin19,HIGH);
    Serial.println("Atras");
    delay(1000);
    digitalWrite(pin2,LOW); digitalWrite(pin18,HIGH);
    digitalWrite(pin4,HIGH);  digitalWrite(pin19,LOW);
    Serial.println("Izquierda");
  }
//  else if (distancia2 < 5){
//    digitalWrite(pin2,HIGH); digitalWrite(pin18,LOW);
//    digitalWrite(pin4,LOW);  digitalWrite(pin19,HIGH);
//    delay(1500);
//    digitalWrite(pin2,LOW); digitalWrite(pin18,LOW);
//    digitalWrite(pin4,HIGH);  digitalWrite(pin19,HIGH);
//  }
  else {
    digitalWrite(pin2,HIGH); digitalWrite(pin18,HIGH);
    digitalWrite(pin4,HIGH); digitalWrite(pin19,LOW);
    Serial.println("Enfrente");
  }
}