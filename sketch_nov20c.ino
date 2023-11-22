// Definir los pines para control del motor
const int pwm = 6;
const int in1  = 5;
const int in2  = 4;

// Definir los pines del Encoder
const int pinA = 3;
const int pinB = 2;

// Variables procesar lecturas del Encoder
volatile int signalB;
volatile long contador = 0;  // poner en 0 cuando angulo sea 0
const int ppr = 600;
float angulo = 0.6*contador;

// Tiempo para estabilizar planta
const int steady_time = 3; // tiempo en segundos
const int tolerancia = 25; // tolerancia para llamar a ManualSetting

// Parametros de PID
double setpoint = 180;
double kp = 9;
double ki = 0.0;
double kd = 0;

// Variables PID
double error, prevError, integral, derivative, output;


void setup() {
  Serial.begin(9600);

  // Configurar los pines de encoder como entrada 
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  // Habilitar las resistencias de pull-up internas
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);

  // Configurar los pines de control de motor como salidas 
  pinMode(pwm, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Tiempo para estabilizar antes de iniciar
  delay(1000*steady_time);
  
  // Configurar la interrupcion para los cambios en el pinA
  attachInterrupt(digitalPinToInterrupt(pinA), actualizarEncoder, RISING);

  //Condicional para llevar la planta manualmente a SetPoint
  manual_set();
}

void loop() {
  PID(); // Llama al bloque de control para calcular el output amplificado
  motor(); // Inyecta el output del PID a la planta (motor)
  
  Serial.print(setpoint); Serial.print(","); Serial.print(angulo); Serial.print(","); Serial.println(abs(output));
  /*Serial.print("   ANGULO >> "); Serial.print(angulo);
  Serial.print(",   ERROR >> "); Serial.print(error);
  Serial.print(",   PID_OUTPUT >> "); Serial.print(output);

  if(output>0){
    Serial.println(",       ||  CLOCKWISE  || ");
  }
  if(output<0){
    Serial.println(",       ||  ANTI-CLOCKWISE  || ");
  }
  if(output==0){
    Serial.println(",       ||  STEADY  || ");
  }*/
}

void PID(){
  error = setpoint - angulo;        // Calcular el error
  
  integral += error;                // Calcular la integral y la derivada

  derivative = error - prevError;   // Calcula la derivada
  
  prevError = error;                // Actualiza la variable error
  
  output = kp * error + ki * integral + kd * derivative;    // Calcular la salida del control PID

  output = constrain(output, -255, 255); // Limita el OUTPUT entre -255 y 255
}

void motor(){
  if(angulo<(setpoint-tolerancia) || angulo>(setpoint+tolerancia)){
    analogWrite(pwm, 0);
    manual_set();
  }
  analogWrite(pwm, abs(output)); //Escribe el valor absoluto del OUTPUT en el pin PWM

  if(output>0){                 // CLOCKWISE
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if(output<0){                 // ANTICLOCKWISE
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }

}

void actualizarEncoder() {  // CALCULO PARA 360 GRADOS
  signalB=digitalRead(pinB);
  if(signalB>0){
    contador++;
  }else{
    contador--;
  }

  /*if(contador>180) contador=contador*-1;
  if(contador==-180) contador=0;
  angulo=0.6*contador;*/
  
  if(contador==600) contador=0;
  if(contador<0) contador=contador+600;
  angulo=0.6*contador;
  //angulo=sin(angulo*(3.1416/180)); // RADIANES
}


void manual_set(){    //Asegura que el sistema comience a trabajar en el SetPoint
    while(angulo!=setpoint){
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
