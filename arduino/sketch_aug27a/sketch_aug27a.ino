const int B_L = 15;
const int B_R = 18;

const int FL = 4;
const int FR = 5;

const int PWMR = 19;
const int PWML = 21;



void setup() {
  pinMode(13,INPUT);
  pinMode(12,INPUT);
  pinMode(14,INPUT);
  pinMode(27,INPUT);
  pinMode(26,INPUT);
  pinMode(25,INPUT);

  pinMode(B_L,OUTPUT);
  pinMode(FL,OUTPUT);

  pinMode(B_R,OUTPUT);
  pinMode(FR,OUTPUT);

  pinMode(PWMR,OUTPUT);
  pinMode(PWML,OUTPUT);

  
  Serial.begin(9600);

}

void loop() {
  int v1 = analogRead(13);
  int v2 = analogRead(12);
  int v3 = analogRead(14);
  int v4 = analogRead(27);
  int v5 = analogRead(26);
  int v6 = analogRead(25);
  Serial.print(v1);
  Serial.print(" ,");
  Serial.print(v2);
  Serial.print(" ,");
  Serial.print(v3);
  Serial.print(" ,");
  Serial.print(v4);
  Serial.print(" ,");
  Serial.print(v5);
  Serial.print(" ,");
  Serial.print(v6);
  Serial.println();
  delay(1000);
  
  digitalWrite(B_L,LOW);

  digitalWrite(FL,HIGH);
  
  analogWrite(PWML,255);

}