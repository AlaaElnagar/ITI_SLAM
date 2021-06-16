
/*This code is for esp8266 Solved with interrupt problem """ICACHE_RAM_ATTR""*/

  const int outA = 0  ; //A+
 
  float vel = 0;
  volatile byte counter = 0;
  volatile byte counter_R = 0;

  float start_time = 0 ;
  float end_time = 0 ;

  float frequancy = 0 ; 
  float frequancy_R ; 
  float n = 0 ;
  float n_R=0;
  float diff_t = 0 ;
  float ang_vel =  0 ; 
  float ang_vel_R =  0 ; 

  float linear_vel = 0 ;
  float linear_vel_R = 0 ;
  float radious = .15 ;

  float rate = .33 ;
  float rate_start = 0 ;
  float rate_end = 0 ;
  float pi = 3.14 ;
  
  
  void ICACHE_RAM_ATTR doEncoderA()  {

counter = counter+1;
counter_R+=1 ;


}

void setup() {
  //define pins 
  pinMode(outA, INPUT_PULLUP);  
  Serial.begin(9600);
  ICACHE_RAM_ATTR;
  attachInterrupt(digitalPinToInterrupt (outA), doEncoderA, RISING);
  //interrupts(); 
  //rate_start = millis() ; 
}



void loop() {
  //rate_end =  millis();
  end_time = millis();
  float diff_t = (end_time - start_time)/1000. ;
 

if ( diff_t > 1){

frequancy= counter/diff_t ;
frequancy_R=counter_R/diff_t ;

n = (frequancy /410)*60. ;

n_R =(frequancy_R /410)*60. ;
 
ang_vel = (2*pi*n)/60. ;    //rad/sec
ang_vel_R=(2*pi*n_R)/60. ; 
linear_vel = ang_vel * radious ;
linear_vel_R = ang_vel_R * radious ;

  start_time = millis();

    Serial.print(linear_vel);
  Serial.print(",");
  Serial.println(linear_vel_R);

  counter = 0 ;
  }


}
