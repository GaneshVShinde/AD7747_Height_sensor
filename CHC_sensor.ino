#include <Wire.h>
#include <LiquidCrystal.h>
#include <EEPROMex.h>
#include <DigitalPin.h>
#define SLAVEWRT 0x48
#define SLAVERD 0x48

DigitalPin<6> _STEPPIN;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
volatile unsigned long step;
volatile unsigned long stepcnt; 
volatile int mode=0;
volatile unsigned start_;

volatile int deacc=0;
volatile int stop;
volatile int halt;

float maxspeed=400;
float accel=4000;
float cmin;
float maxspeednew;
int pulsewidth;
float c0;
float cn;
int cv;

float deadband = 0.2;

int addressFloat = 10;
int Floatarray = 100;


float capvalue2;

float capread();
void rotate();
int count = 0;
float cap;
boolean flag = LOW;
boolean re_tun=LOW;
float capvalue;
int up_limit = 1;
int down_limit =13;
const int upswitch =8;
const int downswitch = 9;
const int autoswitch = 10;
const int pulse = 6;
const int dir = 7;
boolean lastbutton =LOW;
boolean ledon= false;
const int uplimit = 1;
const int downlimit = 13;
unsigned long c = 0;
unsigned long x;
float e;
float cap_array[31];

float tol;
int UPV =0;
int DWNV =0;
boolean cal =LOW;
long lasttime;
long lastcheck;

boolean stat = LOW;
int j =200;

float uper_band;
float lower_band;

void setup() { 
  lcd.begin(16,2);
  Serial.begin(9600);
  pinMode(up_limit,INPUT);
  pinMode(down_limit,INPUT);
  pinMode(pulse,OUTPUT);
  pinMode(dir,OUTPUT);
  pinMode (upswitch,INPUT);
  pinMode (downswitch,INPUT);
  pinMode (autoswitch,INPUT);
  Wire.begin();

  //  lcd.begin(16,2);

  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x07);
  Wire.write(0xA0);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x09);
  Wire.write(0x0E);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x0A);
  Wire.write(0xF9);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x0B);
  Wire.write(0xAB);
  Wire.endTransmission();
  delay(10);
  //cap= EEPROM.readFloat(addressFloat);
  //Serial.print(cap);
  Serial.println("Stored Values");
  for (int i=0;i<31;i++)
  {
    cap_array[i]=EEPROM.readFloat(Floatarray);
    Serial.println(cap_array[i]);
    Floatarray = Floatarray +4;
    //delay(50);
  }

  uper_band=cap_array[20];
  lower_band=cap_array[24];

  // Timer initialising
  cmin = 1000000 / maxspeed;
  c0 = (sqrt(1 / accel)) * 1000000;
  mode=0;
  halt=0;
  cv=0;
  cn=c0;
  step=0;
  stop=0;
  deacc=0;
  stepcnt=0;

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  //dly=cn*2;
  //OCR1A = pulsewidth;
  OCR1A=cn;
  TCCR1B |= (1 << WGM12); // Mode 4, CTC on OCR1A
  //TCCR1B |= (1 << CS12) | (1 << CS10);    // 1024 prescaler
  //TCCR1B |= (1 << CS12);                  // 256 prescaler 
  //TCCR1B |= (1 << CS11) | (1 << CS10);    // 64 prescaler
  TCCR1B |= (1 << CS11);                  // 8 prescaler 
  //TCCR1B |= (1 << CS10);    // no prescaler

  TIMSK1 |= (1 << OCIE1A);

  interrupts();
  halt=1;
  //TIMSK1 = 0;
  lasttime=millis();
  TIMSK1 = 0;

}
void loop() 
{
  int c=0;
  float d;
  int i;
  int j;
  j = 200;
  int b;
  int u;
  float s =0;
  float z = 0.017376;
  float y =0;
  float ly;
  int temp= 0;


  if (digitalRead(upswitch)==LOW&& digitalRead(downswitch)==LOW)
  {
    delay(500);
    if (digitalRead(upswitch)==LOW&& digitalRead(downswitch)==LOW)
      calibration();
  }



  // Change mode / auto / manual
  if (digitalRead(autoswitch)==LOW)
  {
    delay(50);
    if(digitalRead(autoswitch)==LOW && lastbutton == LOW)
    {
      ledon= !ledon;
      lastbutton = HIGH;
      if (ledon == true)
      {
        lcd.setCursor(0,0);
        lcd.print("M ");
        chcreset();
        halt_();

      }
      else
      {
        lcd.setCursor(0,0);
        lcd.print("A ");
        e=capread(500);        
        for (i=0;i<31;i++)
        {
          if (e>cap_array[i] || i==30)
          {
            if (i==30)
              i=29;
            if (i==0)
              i=1;
            lcd.setCursor(0,0);
            lcd.print("Deadbands");
            uper_band=cap_array[i-1];
            lower_band=cap_array[i+1];
            lcd.setCursor(0,1);
            lcd.print(uper_band);
            lcd.setCursor(9,1);
            lcd.print(lower_band);
            delay(1000);
            break;
          }
        }
      }
    }
  }
  else
    lastbutton=LOW;

  // Auto Correction
  e=capread(1000);

  if (ledon==false)
  {
    if (millis()-lastcheck>50)
    {
      lastcheck=millis();
      c=0;
      if (e>uper_band)    // Move torch up
      {
        if (digitalRead(up_limit)!=LOW)
        {
          c=1;
          digitalWrite(dir,LOW);
          //rotate_small();
          //delay(25);
          Start();
        }
      }
      if (e<lower_band) // Move torch down
      {
        if (digitalRead(down_limit)!=LOW)
        {
          c=2;
          digitalWrite(dir,HIGH);
          //rotate_small();
          //delay(25);
          Start();
        }  
      }
      
      if (c==0) // No correction needed, halt timer
      {
        halt_();
      }
    }
  }


  if (lasttime-millis()>300)
  {
    lcd.setCursor(5,0);
    lcd.print(e);
    lasttime=millis();
  }






  if (digitalRead(upswitch)==LOW && digitalRead(downswitch)!=LOW && digitalRead(up_limit)!= LOW )
  {
    lcd.setCursor(0,1);
    lcd.print("up    ");
    delay(50);
    digitalWrite (dir,LOW);
    while (digitalRead(upswitch)==LOW && digitalRead(downswitch)!=LOW && digitalRead(up_limit)!= LOW)
    {
      rotate();

    }


  }




  if(digitalRead(downswitch)==LOW && digitalRead(upswitch)!=LOW && digitalRead(down_limit)!=LOW)
  {
    delay(10);
    lcd.setCursor(0,1);
    lcd.print("down ");
    digitalWrite (dir,HIGH);
    while (digitalRead(downswitch)==LOW && digitalRead(upswitch)!=LOW && digitalRead(down_limit)!=LOW)
    {
      rotate();

    }


  }






}



void rotate()
{
  for (int i =0;i<80;i++)
  {
    digitalWrite(pulse,HIGH);
    delayMicroseconds(j);
    digitalWrite(pulse,LOW);
    delayMicroseconds(j);
  }
}

void rotate_small()
{
  for (int i =0;i<20;i++)
  {
    digitalWrite(pulse,HIGH);
    delayMicroseconds(j);
    digitalWrite(pulse,LOW);
    delayMicroseconds(j);
  }
}



float capread(float x) ////capacitance reading
{
  float d;
  int i;
  int j;
  j = 200;
  int b;
  int u = x;
  float s =0;
  float z = 0.017376;
  float y =0;
  float ly;
  float f;
  float l;
  float capvalue2;
  Wire.beginTransmission(SLAVERD);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(SLAVERD,4);
  for ( u = 0; u<100;u++)
  {
    while(Wire.available())
    {
      c= Wire.read();  // receive high byte (overwrites previous reading)
      c = c << 8;    // shift high byte to be high 8 bits
      c |= Wire.read(); //
      c = c<< 8;
      c |= Wire.read();
      c = c<< 8;
      c |= Wire.read();

      //Serial.println(c,HEX);
      c= c & 0x00FFFFFF;
      float g = c;
      //Serial.print("-");
      //Serial.println(g);
      s = g/1048575;
    }
    y = s+y;
  }
  y = y/u;
  capvalue2 = y;
  return capvalue2; 

}

void calibration()
{  
  capvalue=capread(100);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrate");
  lcd.setCursor(0, 1);
  lcd.print(capvalue);
  int limit;
  int i = 0;
  delay(1000);

  lcd.setCursor(0, 1);
  lcd.print("going down");
  limit=0;
  flag=LOW;
  while (flag ==LOW )
  {
    if (digitalRead(down_limit)==LOW)
    {
      flag=HIGH;
      limit=1;
    }

    digitalWrite(dir,HIGH);
    rotate();
    capvalue = capread(50);
    lcd.setCursor(0, 0);
    lcd.print("           ");
    lcd.setCursor(0,0);
    lcd.print(capvalue);
    if (capvalue>15.0 )
      flag = HIGH;
  }

  if (limit==1)  //Abort calibration because of limit
  {
    lcd.clear();
    lcd.print("LIMIT");
    delay(500);
    return;
    chcreset();
  }

  lcd.clear();
  lcd.print("Going Up");
  delay(500);

  digitalWrite(dir,LOW);
  flag=LOW;
  count=0;

  //Store array
  Floatarray=100;  
  while (flag == LOW)
  {
    delay(100);
    capvalue=capread(50);
    lcd.setCursor(0,0);
    lcd.print("          ");
    lcd.setCursor(0,0);
    lcd.print(capvalue);

    if (capvalue<16)
    {
      cap_array[count]=capvalue;
      delay(25);

      EEPROM.writeFloat(Floatarray,capvalue);
      Floatarray = Floatarray+4;

      lcd.setCursor(0,1);
      lcd.print(count);
      Serial.println(count);
      Serial.print(capvalue);

      count++;
      if (count>30)
        flag=HIGH;


    }
    rotate();

    if (digitalRead(up_limit)==LOW)
    {
      flag=HIGH;
      limit=1;
    }

  }
  lcd.setCursor(0,0);
  lcd.print("Done ");
  delay(1000);
  chcreset();

}


void chcreset()
{
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x07);
  Wire.write(0xA0);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x09);
  Wire.write(0x0E);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x0A);
  Wire.write(0xF9);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(SLAVEWRT);
  Wire.write(0x0B);
  Wire.write(0xAB);
  Wire.endTransmission();
  delay(10);


}


ISR(TIMER1_COMPA_vect)
{ 

  if (mode==0) // High pulse 
  {
    mode=1;
    _STEPPIN.write(HIGH);
    OCR1A=cn;    

  } // End of mode 0


  else        // Low Pulse
  {

    // Deacceleration
    if (deacc==1) // Deacceleration
    {
      _STEPPIN.write(LOW);

      stepcnt=stepcnt-1;
      step=step+1;
      cn = cn + ((2 * cn) / ((4 * (stepcnt)) + 1));
      if (cn>=cmin)      
      {
        deacc=0;
        cn=cmin;
        cv=1;
      }
      mode=0;
      OCR1A=cn;

    } // End of deacceleration

      //  Deacceleration to a halt
    else if (halt==1) 
    {
      _STEPPIN.write(LOW);

      stepcnt=stepcnt-1;
      step=step+1;
      cn = cn + ((2 * cn) / ((4 * (stepcnt)) + 1));
      if (stepcnt<=1)
      {
        TIMSK1 = 0; // Finished deaccelerating. Stop the timer
        stop=1;
        halt=0;
        start_=0;
      }

      mode=0;
      OCR1A=cn;
    } // End od deacceleraion to a halt


    // Acceleration
    else if (cv==0)  
    {
      _STEPPIN.write(LOW);

      stepcnt=stepcnt+1;
      step=step+1;
      cn = cn - ((2 * cn) / ((4 * stepcnt) + 1));

      if (cn<cmin)
      {
        cv=1;
        cn=cmin;
      }
      mode=0;
      OCR1A=cn;
    }  // End of acceleration


      // CONSTANT VELOCITY
    else if (cv==1)
    { // Constant velocity     
      _STEPPIN.write(LOW);

      step=step+1;   
      mode=0;     
      OCR1A=cn;

    } // End of constant velocity

  } // End of mode 1

}


void Start()
{
  if (start_==0)
  {
    start_=1;
    step=0;
    stepcnt=0;

    mode=0;
    cv=0;
    deacc=0;
    halt=0;
    cn=c0;
    //dly=cn*2;
    noInterrupts(); 
    TCNT1  = 0;
    OCR1A=cn;
    TIMSK1 |= (1 << OCIE1A);
    interrupts();  
  }

}

void halt_()
{
                 start_=0;
        TIMSK1 = 0;
        deacc=0;
        cv=0;  
        halt=1;
 
  
}
