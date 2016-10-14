/* ---------------------------------
 * Controls 2 steppers with ESP8266 and CNC shield plus one server on Z step 
 * for 4xiDraw machine  
 *  
 * code by misan, October 2016
 * known to work on Wemos D1 board 
 *  
 * both serial 115200 and UDP 9999 accept g-code 
 */  

#define MAXI 30
#define VERSION        (1)  // firmware version
#define BAUD           (115200)  // How fast is the Arduino talking?
#define MAX_BUF        (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN (400)  // depends on your stepper motor.  most are 200.
#define MAX_FEEDRATE   (50000) 
#define MIN_FEEDRATE   (0.01)

#define STEPS_MM  80


int ACCEL =    1500 ;  // mm/s^2

boolean flip = false;
// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (1)

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
const char* SSID_NAME = "ESP8266AP";  // WiFi AP SSID Name
const char* SSID_PASS = "0123456789"; // WiFi AP SSID Password
WiFiUDP port;
unsigned int localPort = 9999;

WiFiServer server(9999);
WiFiClient client;

int i = 1;

long last=0;
int servo=1500;

float factor, v2 = 0;


// interrupt triggered by timer expiring
// motion will happen here based on a few variables

volatile int cs=0; // current step
long accel_steps=1000; // how many steps during acceleration
long coast_steps=40000; // how many steps during coasting
long total_steps=0; // total number of steps for the current movement
float t0=0.1;   // determines initial speed
float cn;
boolean diry,dirx;
long over;
float dx,dy;

volatile boolean busy = false;

void itr1(void) {
  if(flip) {
    digitalWrite(D14,LOW);
    flip = false;
    timer1_write(20000 * 5); // 20 ms between pulses
  }
  else {
    timer1_write(servo*5);
    flip = true;
    digitalWrite(D14,HIGH);
  }
}

void itr (void) {
  //digitalWrite(BUILTIN_LED,!digitalRead(BUILTIN_LED));
  if( cs >= total_steps ) {busy = false; timer0_write(ESP.getCycleCount() + 80000000L/100);  } // no more interrupts scheduled
  else { // I have to move the motors
    cs++;
    if( cs < accel_steps ) {
      // acceleration
      cn = cn - (cn*2)/(4*cs+1);
      //Serial.println(cn);
    }
    else if ( cs < accel_steps + coast_steps ) {
    // coast at max speed
    }
    else if(cs<total_steps) {
      // decceleration
      cn = cn - (cn*2)/(4*(cs-total_steps)+1); 
      //Serial.println(cn);
    }
    
    timer0_write(ESP.getCycleCount() + (long) (20000000L*cn) );  // schedule next pulse

    if(dx>dy) {
      over=dx/2;
      digitalWrite(D2, HIGH);
      over+=dy;
      if(over>=dx) {
        over-=dx;
        digitalWrite(D15, HIGH);
      }
      delayMicroseconds(1);
    } else {
    over=dy/2;
      digitalWrite(D15, HIGH);
      over+=dx;
      if(over>=dy) {
        over-=dy;
        digitalWrite(D2, HIGH);
      }
      delayMicroseconds(1);
  }
  digitalWrite(D15, LOW);
  digitalWrite(D2 , LOW);   
  } 
}

//------------------------------------------------------------------------------
// 2 Axis CNC Demo
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

char buffer[MAX_BUF];  // where we store the message until we get a newline
int sofar;  // how much is in the buffer

float px, py;  // location

// speeds
float fr=1000;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.
  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr=nfr;
  cn = t0 = sqrt( 2.0 /** STEPS_MM*/ / ACCEL );
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}

//#define max(a,b) ((a)>(b)?(a):(b))

/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {
  dx=newx-px;
  dy=newy-py;
  dirx=dx>0;
  diry=dy>0;  // because the motors are mounted in opposite directions
  dx=abs(dx);
  dy=abs(dy);
// Serial.print("line: ");Serial.print(dx); Serial.print(","); Serial.println(dy);
//  move(dx,dirx,dy,diry);
  long i;
  over=0;

  total_steps = _max ( dx, dy ) * STEPS_MM; 
  float ta = fr / 60.0 / ACCEL;
  float ea = ( fr / 60.0 ) * ta / 2 ;
  accel_steps = _min(ea * STEPS_MM , total_steps/2); // just in case feedrate cannot be reached 
  coast_steps = total_steps - accel_steps * 2; // acceleration
  digitalWrite(D13, dirx); // direction of X motor
  digitalWrite(D12, diry);
  cs = 0; // let the motion start :-)
  cn = t0;

  if(dx>dy) over=dx/2; else over=dy/2;
  busy=true;
  // wait till the command ends
  while(busy) { /*doServo();*/ delay(0); }
  px=newx;
  py=newy;
}


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }
  
  line(x,y);
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;
  while(ptr && *ptr && ptr<buffer+sofar) {
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr++; //ptr=strchr(ptr,' ')+1;
  }
  return val;
} 


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo2AxisV1 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
  // Print the IP address
  Serial.print("Use this URL to connect: http://");
  Serial.print(WiFi.softAPIP());
  Serial.print(':');
  Serial.print(localPort);
  Serial.println('/');
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parsenumber('G',-1); 
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parsenumber('F',fr));
      arc(parsenumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0) );
    break;
  default:  break;
  }

  cmd = parsenumber('M',-1); 
  switch(cmd) {
  case 3:servo = ( parsenumber('S',1500) ); break; // sets the servo value in microseconds, it only works while inside loop :-(
  case 18:  // disable motors
//    disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  case 201: ACCEL = parsenumber('A',0); feedrate(fr); Serial.print(ACCEL); break; // M201 A<accel> change acceleration
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}




void setup() {
  Serial.begin(BAUD);  // open coms
  
  pinMode(D2, OUTPUT);  // stepX
  pinMode(D13, OUTPUT); //dirX
  pinMode(D8, OUTPUT); // enable
  pinMode(D15, OUTPUT); // stepY
  pinMode(D12, OUTPUT); // dirY
  pinMode(D14, OUTPUT); // servo
  //pinMode(BUILTIN_LED,OUTPUT);
  digitalWrite(D8, LOW);
  digitalWrite(D14, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID_NAME, SSID_PASS);
  port.begin(localPort);

  //  setup_controller();  
  position(0,0);  // set staring position
  feedrate((MAX_FEEDRATE + MIN_FEEDRATE)/2);  // set default speed

  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(itr);
  timer0_write(ESP.getCycleCount() +5000);

  timer1_isr_init();
  timer1_attachInterrupt(itr1);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(servo*5);
  interrupts();

  server.begin(); // start TCP server

  help();  // say hello
  ready();
}


char c;
long lastping;

void loop() {
  /*
  int packetSize = port.parsePacket();
  if (packetSize) {
    int len = port.read(buffer, 255); sofar=len; 
    if (len > 0) buffer[len-1] = 0; 
    Serial.println(buffer);
    processCommand();
    port.beginPacket(port.remoteIP(),port.remotePort());
    port.write("ok\r\n");
    port.endPacket();
  }
  */

 if(millis()-lastping>5000 && !client) { // only till first client connects
  lastping=millis();
  port.beginPacket("255.255.255.255",9999);
  port.write("Anybody there?\r\n");
  port.endPacket();
 }
  
  while(Serial.available() > 0 ) {  // if something is available
    c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if((c=='\n') || (c == '\r')) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      if(sofar>3) {
                  busy=true;
                  processCommand();  // do something with the command
                  ready(); 
                  }
    } 
  } 
  if(server.hasClient()){
    client=server.available();  // it will destroy a previous connection !!!
    Serial.println("TCP Client accepted");
  }
  if(client && client.connected()) {
    while(client.available()>0) {// if data available,let's hope it is the full command
      buffer[sofar++]=(c=client.read()); Serial.print(c); 
      if((c=='\n') || (c == '\r')) {
        buffer[sofar]=0;
        busy=true;
        processCommand();  // do something with the command
        ready(); 
        client.println("ok");
      }
    }
  }
}


