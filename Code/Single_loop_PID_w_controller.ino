float kp = 0.000000002523;
float ki = 0.019847;
float kd = -0.0000159625;

unsigned long t;
unsigned long t_prev = 0;

const byte encoderA = 2;
const byte encoderB = 3;
const byte en_PWM = 5;
const byte mot_p = 6;
const byte mot_n = 4;

volatile unsigned long timerCounter = 0;
unsigned long timerCounter_prev = 0;
volatile long encCount = 0;

float theta, theta_prev = 0;
float RPM, RPM_prev=0, RPM_r;
float a=0;
int dt;

#define pi 3.1416
#define CPR 48
float RPM_max = 7000;
float Vmax = 12;
float V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0, deriv=0;

/***********FUNCTIONS******************
       Void ISR_EncA
       Void ISR_EncB
       Void MotorDriver
       Timer Interrupt
***************************************/

void ISR_EncA()
{
  bool valB = digitalRead(encoderB);
  bool valA = digitalRead(encoderA);
  
  if(valA^valB)
  {
    encCount++;
  }
  else
  {
    encCount--;
  }
}

void ISR_EncB()
{
  bool valA = digitalRead(encoderA);
  bool valB = digitalRead(encoderB);

  if(valA^valB)
  {
    encCount--;
  }
  else
  {
    encCount++;
  }
}


float sgn(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

//*****Motor Driver Function*****//

void MotorDriver(float V, float Vmax) {
  int PWM = int(255 * abs(V) / Vmax);
  if (PWM > 255) {
    PWM = 255;
  }
  if (V > 0) {
    digitalWrite(mot_p, HIGH);
    digitalWrite(mot_n, LOW);
  }
  else if (V < 0) {
    digitalWrite(mot_p, LOW);
    digitalWrite(mot_n, HIGH);
  }
  else {
    digitalWrite(mot_p, LOW);
    digitalWrite(mot_n, LOW);
  }
  analogWrite(en_PWM, PWM);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), ISR_EncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), ISR_EncB, CHANGE);
  pinMode(en_PWM, OUTPUT);
  pinMode(mot_p, OUTPUT);
  pinMode(mot_n, OUTPUT);

  Serial.begin(9600);

//-------------- Setting-up TIMER1 --------------//
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 249; //delta time of 1ms.
  TCCR1B |= (1 << WGM12);  //CTC Mode.
  TCCR1B |= (1 << CS11 | 1 << CS10);  //Prescaler = 64.
  TIMSK1 |= (1 << OCIE1A);  //enabling output compare interrupts.
  sei();
}

void loop() {
  if (timerCounter > timerCounter_prev) {
    t = millis();
    theta = encCount / CPR;
    dt = (t - t_prev);
    
//    RPM_d = 5500;
    RPM_r = RPM_max * (sin(2 * pi * 0.005 * t / 1000.0)) * sgn(sin(2 * pi * 0.05 * t / 1000.0));
    if (t / 1000.0 > 100) {
      RPM_r = 0;
    }
    
    RPM = (theta - theta_prev) / (dt / 1000.0) * 60;
    e = RPM_r - RPM;
    inte = inte_prev + ((dt/1000.0) * (e + e_prev) / 2.0);
    deriv = (e - e_prev) / (dt/1000.0);
    V = kp * e + ki * inte + (kd * deriv) ;
    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < -Vmax) {
      V = -Vmax;
      inte = inte_prev;
    }

    a = ((RPM - RPM_prev) * 2*pi/60.0) / (dt/1000.0);
    
    MotorDriver(V, Vmax);

//    Serial.print(a);
//    Serial.print(",");
    Serial.print(RPM_r); 
    Serial.print(",");
    Serial.print(RPM); 
    Serial.print(",");
    Serial.print(RPM_max); 
    Serial.print(",");
    Serial.println(-RPM_max);

    theta_prev = theta;
    RPM_prev = RPM;
    timerCounter_prev = timerCounter;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }
}



ISR(TIMER1_COMPA_vect) {
  timerCounter++;
//  Serial.print(count * 0.05); Serial.print(" \t");
}
