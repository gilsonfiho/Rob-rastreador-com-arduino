int PinoVelocidade = 3; //Ligado ao pino 1 do L293D  
int Entrada1 = 2; //Ligado ao pino 2 do L293D  
int Entrada2 = 7; //Ligado ao pino 7 do L293D  

void para_motor() {  
  digitalWrite(Entrada1, LOW);  
  digitalWrite(Entrada2, LOW);  
  delay(3000);  
} 
   
void setup()  {  
  //Define os pinos como saida  
  pinMode(PinoVelocidade, OUTPUT);  
  pinMode(Entrada1, OUTPUT);  
  pinMode(Entrada2, OUTPUT);  
}  
   
void loop()  {  
  //Define a velocidade de rotacao  
  int velocidade = 500;  
  analogWrite(PinoVelocidade, velocidade);   

  //Aciona o motor  
  digitalWrite(Entrada1, LOW);  
  digitalWrite(Entrada2, HIGH);  
  delay(3000);  

  //Chama a rotina de parada do motor  
  para_motor();  

  //Aciona o motor no sentido inverso  
  digitalWrite(Entrada1, HIGH);  
  digitalWrite(Entrada2, LOW);  
  delay(3000);  
  para_motor();  
}  
    

