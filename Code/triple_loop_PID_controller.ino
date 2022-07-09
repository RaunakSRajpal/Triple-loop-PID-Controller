#include<avr/io.h>

#define encoderA 2
#define encoderB 3
#define en_PWM 5
#define mot_p 6
#define mot_n 4
#define isoAmpRead A0
#define potPin A1
#define ledPin 7
#define switchPin 8

//------control parameters-------//
const float Kp_t = 0.00000000237;
const float Ki_t = 0.002756;
const float Kd_t = 0.0147;

const float Kp_w = 0.00000000001723;
const float Ki_w = 0.065047;
const float Kd_w = -0.000175725;

const float Kp_i = 0.00010;
const float Ki_i = 1.00055;
const float Kd_i = 0.0455;

#define pi 3.1416
#define CPR 48  // PPR=12 | CPR = 4xPPR.
#define Vmax 12
#define Imax 6
#define max_RPM 7000
#define a 10000  //rad/s^2 | considering acceleration of the motor to be const. at its max value.

//------variables------//
volatile bool ledState = false;
volatile bool switchVal = false;
volatile bool modeVal = false;  //theta-mode = 1, w-mode = 0.

volatile long encCount=0;

unsigned long t, t_prev=0, dt=0;

volatile unsigned long timerCounter=0, timerCounter_prev=0;

float theta, theta_prev=0;
float theta_r, theta_r_prev=0;
float theta_e, theta_e_prev=0;
float w=0, w_r=0;
float w_e, w_e_prev=0;
float v_read = 0, i=0, i_r=0;
float i_e, i_e_prev=0;
int PWM=0;

float inte_t, inte_t_prev=0, deriv_t;
float inte_w, inte_w_prev=0, deriv_w;
float inte_i, inte_i_prev=0, deriv_i;
float u_t=0.1;
float V=0.1;


ISR(TIMER1_COMPA_vect)
{
  timerCounter++;
}


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


void MotorDriver(float V)
{
  PWM = (abs(V)/Vmax)*255;
  analogWrite(en_PWM, PWM);
  if(V>0)
  {
    digitalWrite(mot_p, HIGH);
    digitalWrite(mot_n, LOW);
  }
  else if(V<0)
  {
    digitalWrite(mot_p, LOW);
    digitalWrite(mot_n, HIGH);
  }
  else
  {
    digitalWrite(mot_p, HIGH);
    digitalWrite(mot_n, HIGH);
    delayMicroseconds(100);
    digitalWrite(mot_p, LOW);
    digitalWrite(mot_n, LOW);
  }
}


void TVPA(float theta_r, unsigned long t)
{
  float t_ac_tri = 1000.0 * sqrt((theta_r * 2.0*pi/360.0) / a);
  float t_ac_trpz = 1000.0 * ((max_RPM * 2*pi/60.0) / a);
  float T = 1000.0 * ((theta_r * 2.0*pi/360.0)/(max_RPM * 2*pi/60.0));   // T = t_ac_trpz + t_const

  if(t_ac_tri < t_ac_trpz)
  {
    if(t < t_ac_tri)
      w_r = a*t;
    else
      w_r = a*(t_ac_tri - t);
  }
  else
  {
    if(t < t_ac_trpz)
      w_r = a*t;
    else if(t >= T && t < (T+t_ac_trpz))
      w_r = max_RPM - a*t;
    else
      w_r = max_RPM;
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



void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), ISR_EncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), ISR_EncB, CHANGE);

  pinMode(en_PWM, OUTPUT);
  pinMode(mot_p, OUTPUT);
  pinMode(mot_n, OUTPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  
  Serial.begin(9600);

//--------- Setting-up PinChange Interrupts ---------//
  PCIFR = 0x00;
  PCMSK0 |= (1 << PCINT2);   //digitalPin 10(PCINT2) acts as an interrupt for Toggle Switch.
  PCICR |= (1 << PCIE0);

//-------------- Setting-up TIMER1 --------------//
  cli();
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCNT1 = 0x00;
  TCCR1B |= (1<<WGM12 | 1<<CS11 | 1<<CS10);  //CTC Mode, prescaler = 64. 
  OCR1A = 249;   //delta time of 1ms.
  TIMSK1 |= (1<<OCIE1A); //enabling output compare interrupts. 
  sei();
}


void loop() {
  // put your main code here, to run repeatedly:
  if(timerCounter > timerCounter_prev)
  {
    t=millis();
    theta = (encCount/CPR) * 360.0;  //no. of revs * 360deg.
    dt = t - t_prev;

    if(modeVal == 1)
    {
      theta_r = 100;
//      theta_r = 360 * (sin(2 * pi * 0.005 * t / 1000.0)) * sgn(sin(2 * pi * 0.05 * t / 1000.0));
      if (t / 1000.0 > 100) {
        theta_r = 0;
      }
      theta_r = theta_r - theta_r_prev;

      TVPA(theta_r, t);


    //------------PID-outer-loop------------//
    theta_e = theta_r - theta;
    inte_t = inte_t_prev + (0.5 * (theta_e + theta_e_prev) * (dt/1000.0));
    deriv_t = (theta_e - theta_e_prev) / (dt/1000.00);
    
    u_t = (Kp_t * theta_e) + (Ki_t * inte_t) + (Kd_t * deriv_t);  //output RPM values.

    if (u_t > max_RPM) {
      u_t = max_RPM;
      inte_t_prev = inte_t;
    }
    if (u_t < -max_RPM) {
      u_t = max_RPM;
      inte_t_prev = inte_t;
    }

      w_r += u_t;
    }
    else
    {
//      w_r = 6000;
      w_r = max_RPM * (sin(2 * pi * 0.005 * t / 1000.0)) * sgn(sin(2 * pi * 0.05 * t / 1000.0));
      if (t / 1000.0 > 100) {
      w_r = 0;
      }
    }
    
    w = ((theta - theta_prev) / 360.0) / (dt / 1000.0) * 60;   //units in RPM.


    //------------PID-middle-loop------------//
    w_e = w_r - w;
    inte_w = inte_w_prev + ((dt/1000.0) * (w_e + w_e_prev) * 0.5);
    deriv_w = (w_e - w_e_prev) / (dt/1000.0);
    
    i_r = (Kp_w * w_e) + (Ki_w * inte_w) + (Kd_w * deriv_w) ;  //output voltage function.
    
    if (i_r > Imax) {
      i_r = Imax;
      inte_w = inte_w_prev;
    }
    if (i_r < -Imax) {
      i_r = -Imax;
      inte_w = inte_w_prev;
    }

    v_read = analogRead(isoAmpRead);
    i = ((250.0 / 1023.0) * v_read) * sgn(w);


    //------------PID-inner-loop------------//
    i_e = i_r - i;
    inte_i = inte_i_prev + ((dt/1000.0) * (i_e + i_e_prev) * 0.5);
    deriv_i = (i_e - i_e_prev) / (dt/1000.0);
    
    V = (Kp_i * i_e) + (Ki_i * inte_i) + (Kd_i * deriv_i) ;  //output voltage function.
    
    if (V > Vmax) {
      V = Vmax;
      inte_i = inte_i_prev;
    }
    if (V < -Vmax) {
      V = -Vmax;
      inte_i = inte_i_prev;
    }


    MotorDriver(V);


/*    Serial.print(theta_r+theta_r_prev);
    Serial.print(",");
    Serial.println(theta);
    Serial.print(",");*/
    Serial.print(w_r);
    Serial.print(",");
    Serial.print(w);
    Serial.print(",");
    Serial.print(max_RPM);
    Serial.print(",");
    Serial.print(-max_RPM);
    Serial.print(",");
    Serial.print(i_r);
    Serial.print(",");
    Serial.println(i);
/*    Serial.print(modeVal);
    Serial.print(",");
    Serial.println(ledState);

    digitalWrite(ledPin, ledState);*/
    
    //****** variable updation *****//
    t_prev = t;
    
    theta_prev = theta;
    theta_r_prev = theta_r;
    theta_e_prev = theta_e;
    inte_t_prev = inte_t;
    
    w_e_prev = w_e;
    inte_w_prev = inte_w;
    
    i_e_prev = i_e;
    inte_i_prev = inte_i;
    
    timerCounter_prev = timerCounter;
  }
}

//******** Toggle Switch ISR ********//
ISR(PCINT0_vect)
{
  switchVal = digitalRead(switchPin);
  if(!switchVal)
  {
    ledState ^= true;
    modeVal ^= true;  //theta-mode = 1, w-mode = 0.
  }
}
