// Robô: GF-18
// Autor: Gilson Filho
// O programa tem como objetivo fazer o robô andar autonomamente, desviando de obstaculos, através de um sensor ultrassonico;

//Declarando bibliotecas necessarias 
#include<Ultrasonic.h> // Biblioteca do sensor ultrassonico
#include<Servo.h> // Biblioteca para o servo motor

//Definindo portas digitais para o Sensor ultrassonico 
#define echoPin 13 // Porta de recebimento do eco da onda atraves do echo
#define trigPin 12 // Porta de envio da onde atraves do trig

//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(trigPin,echoPin);

//Declarando o objeto para o servo motor
Servo myservo;

//Declarando o pino do servo motor
#define pinServo 8

//Pinos do Farol
#define pinLdr A0 
#define pinLed1 10
#define pinLed2 11
int pLdr;



//Pino do buzzer
#define pinBuzzer A2


//Variaveis auxiliares para o sensor ultrassonico
long microsec = 0;
float distanciaCM = 0;
int distancia = 0;
int distancia1 = 0;
int distancia2 = 0;

//Variaveis para controlar a velocidade dos motores
int velocidadeA = 6;
int velocidadeB = 3;
#define pinPot A0
int b;
int c;

//Variaveis dos motores
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
 
void setup(){

  //Definindo a frequencia de dados do monitor serial
  Serial.begin(9600);

  //Ligando a porta digital do servo 
  myservo.attach(pinServo);

  //Ligando as portas digitais de saida dos motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Ligando as portas digitais de saida para controle de velocidade
  pinMode(velocidadeA, OUTPUT);
  pinMode(velocidadeB, OUTPUT);

  //Ligando as portas digitais de entrada e saida do sensor ultrassonico
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT); 

  //Ligando as portas do farol
  pinMode(pinLed1,OUTPUT);
  pinMode(pinLed2,OUTPUT);

  //Ligando a porta do buzzer
  pinMode(pinBuzzer,OUTPUT);

}
 
void loop(){
  
  myservo.write(90); //Servo na posiçao 90 (Frente)
  motorFrente(); // Robô anda pra frente
  delay(100); // Espera 100 microsegundo
  distancia = ultrasonico(); // Verifica a distancia do obstaculo a frente
  Serial.println(distancia); // Imprimi a distancia em cm no Monitor serial
  delay(10); // Espera 10 microsegundos

  pLdr = analogRead(pinLdr);
  delay(10);
  Serial.print("pLdr" );
  Serial.println(pLdr);
  delay(10);

  farol(); //Liga o farol caso tenha pouca luminosidade
  
  if(distancia <= 10){ // Se a distancia do obstaculo for menor ou igual

    digitalWrite(pinBuzzer,HIGH);//Aciona o Buzzer
    motorRe(); // Motor vai para trás, para dimunuir a inercia e impedir que haja uma colisão com o obstaculo
    delay(350); // Espera 350 microsegundos
    digitalWrite(pinBuzzer,LOW);//Desliga o Buzzer
    motorFreio(); // Robô para
    myservo.write(0); // Servo motor vai para a posição 0 (Esquerda)
    distancia = ultrasonico(); // Verifica a distancia do obstaculo a esquerda
    Serial.println(distancia); //Imprimi a distancia em cm no Monitor serial 
    delay(10); // Espera 10 microsegundos
    myservo.write(180); // Servo motor vai para a posição 180 (Direita)
    distancia1 = ultrasonico(); // Verifica a distancia do obstaculo a direita
    Serial.println(distancia); //Imprimi a distancia em cm no Monitor serial 
    delay(10); // Espera 10 microsegundos
    myservo.write(0); // Servo motor vai para a posição 180 (Direita)
    distancia2 = ultrasonico(); // Verifica a distancia do obstaculo a esquerda
    Serial.println(distancia); //Imprimi a distancia em cm no Monitor serial 
    delay(10); // Espera 10 microsegundos
    
    if(distancia1 <= distancia2){  // Se a distancia da esquerda for maior que da direita
      
      motorEsquerda(); // Robô vai para a esquerda
      delay(500); // Espera 500 microsegundos
      
    }
    else{ // Se nao
      
      motorDireita(); // Robô vai para a direita
      delay(500); // Espera 500 microsegundos
      
    }
  }
  
}


//Funçao para o robô ir para frente + controle de velocidade
void motorFrente(){

    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(velocidadeA,b); // Velocidade em 138/255
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(velocidadeB,c); // Velocidade em 100/255
    
}

//Funçao para o robô dar re + controle de velocidade
void motorRe(){

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(velocidadeA,100); // Velocidade em 188/255
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(velocidadeB,100); // Velocidade em 150/255
    
}

//Funçao para o robô ir para esquerda + controle de velocidade
void motorDireita(){
    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(velocidadeA,100); // Velocidade em 138/255
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(velocidadeB,100); // Velocidade em 100/255
    
}

//Funçao para o robô ir para direita + controle de velocidade
void motorEsquerda(){
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(velocidadeA,100); // Velocidade em 138/255
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(velocidadeB,100); // Velocidade em 100/255
    
}

//Funçao para o robô parar
void motorPara(){
  
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    
}

void motorFreio(){

    analogWrite(velocidadeA,255);
    analogWrite(velocidadeB,255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    
  
}

//Função para medir a distancia de um obstaculo
int ultrasonico(){

  digitalWrite(trigPin, LOW); // Desliga a porta digital do Trigpin
  delayMicroseconds(2); // Espera 2 microsegundos
  digitalWrite(trigPin, HIGH); // Liga a porta do Trigpin
  delayMicroseconds(10); // Espera 10 microsegundos
  digitalWrite(trigPin, LOW); // Desliga a porta digital do Trigpin
  microsec = ultrasonic.timing(); // Le as informações do sensor
  int distancia = ultrasonic.convert(microsec, Ultrasonic::CM); // Converte a distancia em CM
  return(distancia); // Retorna o valor da distancia

}

int farol(){

  if(pLdr < 50){ //Se o nível de luminosidade do ldr for maior do que 820
    
    digitalWrite(pinLed1,HIGH); //Liga o led 1
    digitalWrite(pinLed2,HIGH); //Liga o led 2
    
  }
  else{ //Se não
    
    digitalWrite(pinLed1,LOW); //Desliga o led 1 
    digitalWrite(pinLed2,LOW); //Desliga o led 2
    
  }
    
}
