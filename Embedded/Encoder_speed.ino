 #define outA 2 //A+
 
  float vel = 0;
  volatile byte counter = 0;
  float start_time = 0 ;
  float end_time = 0 ;

  float frequancy = 0 ; 
  float n = 0 ;
  float diff_t = 0 ;
  float ang_vel =  0 ; 
  float linear_vel = 0 ; 
  float radious = .15 ;

  float rate = .33 ;
  float rate_start = 0 ;
  float rate_end = 0 ;
  float pi = 3.14 ;
void setup() {
  //define pins 
  pinMode(outA, INPUT_PULLUP);  
  Serial.begin(9600);

  attachInterrupt(0, doEncoderA, RISING);
  interrupts(); 
  //rate_start = millis() ; 
}

void loop() {
  //rate_end =  millis();
  end_time = millis();
  float diff_t = (end_time - start_time)/1000. ;
 

if ( diff_t > .03){

  frequancy= counter/diff_t ;

  
n = (frequancy /410)*60. ;


 
ang_vel = (2*pi*n)/60. ;    //rad/sec
linear_vel = ang_vel * radious ;
  
  start_time = millis();

    Serial.print(linear_vel);
  Serial.print(",");
  Serial.println(ang_vel);
  //rate_start = micros();

  
  counter = 0 ;
  }

//update rate 
//float rate_diff = (rate_end - rate_start) /1000000;
/*
if (rate_diff > rate )
{
  Serial.print(linear_vel);
  Serial.print(",");
  Serial.println(linear_vel);
  rate_start = micros();
}

*/
}

void doEncoderA()  {

counter = counter+1;



 

}
