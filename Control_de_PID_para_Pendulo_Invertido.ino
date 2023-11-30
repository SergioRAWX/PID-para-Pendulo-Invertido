//Lectura potenciometro auxiliar para sintonizar manualmente
const double pot = A0;
double k = 1;
double maximo = 16; //Valor maximo para k

// Definir los pines para control del motor
const int pwm = 6;
const int in1  = 5;
const int in2  = 4;

// Definir los pines del Encoder
const int pinA = 3;
const int pinB = 2;

// Variables procesar lecturas del Encoder
volatile int signalB;
const int cuentas_iniciales = 0;
volatile long contador = cuentas_iniciales;  // poner en 0 cuando angulo sea 0
const int ppr = 600;
float angulo = 0.6*contador;
const int boton = 10; //reset de contador

// Tiempo para estabilizar planta
const int steady_time = 2; // tiempo en segundos
const int tolerancia = 30; // tolerancia para llamar a ManualSetting--------------------------


// Parametros de PID
double setpoint = 180;
//----------------- VALORES CON SUBAMORTIGUAMIENTO
double kp = 11.75; //  11.5 
double ki = 3.95;   //  4.2-3.9
double kd = 2.0;  //  2.0-2.8

//Parapemtros proporcionales del motor para igualar potencia en el sentido de giro
double output_limit = 220 ;
double kckw = 1.085;
double kantckw = 1.105;

//AJUSTE EN EJE VERTICAL PARA LAS GRAFICAS DEL SERIAL PLOTTER
int ajustegrafico = 25+35; //-----------------------

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
  analogWrite(A1, 1023);

  // Configurar los pines de control de motor como salidas 
  pinMode(pwm, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Tiempo para estabilizar antes de iniciar
  delay(1000*steady_time);
  
  // Configurar la interrupcion para los cambios en el pinA
  attachInterrupt(digitalPinToInterrupt(pinA), actualizarEncoder, RISING);

  //boton para resetear cuentas
  pinMode(boton, INPUT);
  pinMode(10, INPUT);
  digitalWrite(10, HIGH);
  //Condicional para llevar la planta manualmente a SetPoint
  manual_set();
}




void loop() {
  PID(); // Llama al bloque de control para calcular el output amplificado
  motor(); // Inyecta el output del PID a la planta (motor)

  //SE IMPRIMEN LOS VISUALES PARA SERIAL PLOTTER
  Serial.print(setpoint-ajustegrafico); Serial.print(","); Serial.print(angulo-ajustegrafico); Serial.print(","); Serial.print((abs(output)/10)+25.5); Serial.print(",");   // DATOS DE CONTROL
  Serial.print(setpoint-tolerancia-ajustegrafico); Serial.print(","); Serial.print(setpoint+tolerancia-ajustegrafico); Serial.print(",");  //LIMITES DE ERROR
  Serial.print(510/10);Serial.print(","); Serial.println(255/10);  // LIMITES DE SATURACION +-25

}




//BLOQUE PROPORCIONAL PID
void PID(){
  error = setpoint - angulo;        // Calcular el error
  
  integral += error;                // Calcular la integral y la derivada

  derivative = error - prevError;   // Calcula la derivada
  
  prevError = error;                // Actualiza la variable error
  
  output = kp * error + ki * integral + kd * derivative;    // Calcular la salida del control PID

  output = constrain(output, -output_limit, output_limit); // Limita el OUTPUT entre -255 y 255

}


//SENTIDO DE GIRO E INYECCION DE PWM
void motor(){
  if(angulo<(setpoint-tolerancia) || angulo>(setpoint+tolerancia)){
    analogWrite(pwm, 0);
    manual_set();
  }

  if(output>0){                 // CLOCKWISE
    analogWrite(pwm, abs(kckw*output)); //Escribe el valor absoluto del OUTPUT en el pin PWM
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if(output<0){                 // ANTICLOCKWISE
    analogWrite(pwm, abs(kantckw*output)); //Escribe el valor absoluto del OUTPUT en el pin PWM
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }

}





void actualizarEncoder() {  // CONVERSION DE CUENTAS PPR PARA 360 GRADOS (RESOLUCION DE 0.6grados POR PULSO)
  signalB=digitalRead(pinB);
  if(signalB>0){
    contador++;
  }else{
    contador--;
  }

  if(contador==600) contador=0;
  if(contador<0) contador=contador+600;
  angulo=0.6*contador;
  //angulo=sin(angulo*(3.1416/180)); // CONVERSION A RADIANES
}





void manual_set(){            //Asegura que el sistema comience a trabajar en el SetPoint
    while(angulo!=setpoint){
    //reset_to_cero();
    //Serial.print("0");
    digitalWrite(LED_BUILTIN, HIGH);
    error=0;
    output=0;
    prevError=0;
    integral=0;
    derivative=0;
    
  }
}
