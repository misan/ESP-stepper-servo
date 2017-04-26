int servo=1000; // number of microseconds to set the servo arm (800..2100) 
bool flip=false;
int outPin=D11;

void servo_init() {
  pinMode(outPin,OUTPUT);
  noInterrupts();
  timer1_isr_init();
  timer1_attachInterrupt(itr1);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(servo*5);
  interrupts();
}

void itr1(void) {
  if(flip) {
    digitalWrite(outPin,LOW);
    flip = false;
    timer1_write(20000 * 5); // 20 ms between pulses
  }
  else {
    timer1_write(servo*5);
    flip = true;
    digitalWrite(outPin,HIGH);
  }
}

