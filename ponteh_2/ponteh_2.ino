#define IN1 4
#define IN2 5
#define IN3 8
#define IN4 9

#define velocidadeA 11
#define velocidadeB 10

#define pinBot1 2
#define pinBot2 7



void motorFrente(){

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(velocidadeA,100); //Controle de velocidade do motor
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(velocidadeB,100); //Controle de velocidade do motor
    
}

void motorRe(){

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(velocidadeA,100); //Controle de velocidade do motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(velocidadeB,100); //Controle de velocidade do motor
    
}

void motorPara(){
  
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    
}



void setup() {
  
  //Ligando as portas digitais de saida dos motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Ligando as portas digitais de saida para controle de velocidade
  pinMode(velocidadeA, OUTPUT);
  pinMode(velocidadeB, OUTPUT);

  //Ligando as portas dos botões
  pinMode(pinBot1,INPUT_PULLUP);
  pinMode(pinBot2,INPUT_PULLUP);

  Serial.begin(9600);
  

}

void loop() {

  int x = digitalRead(pinBot1); // Apertado = 0
  int y = digitalRead(pinBot2); // Apertado = 0
  Serial.println(x);
  Serial.println(y);

  if( x == 0 ){
    motorFrente();
  }
  else if( y == 0 ){
    motorRe();
  }
  else{
    motorPara();
  }
  
 
}
