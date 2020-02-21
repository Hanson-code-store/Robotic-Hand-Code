/****************************************************
* Robot Hand Control                                *
* By: Tom Hanson (tomhanson13@gmail.com             *
* Date: 20-10-2019                                  *
*                                                   *
*                                                   *
* Inputs: -EMG sensor(EMG_PIN): activates the hand  * 
*         via an EMG signal                         *
*         -EMG toggle switch(EMG_MODE_PIN):         *
*         determines the EMG mode(Latch or          *
*         continuious)                              *
*         -Potentiometer(POT): sets maximum grip    *
*         strength                                  *
*         -Force Sensors(FORCE_): Analog signal     *
*         from 3 force resistice sensors            *
*         -Mode switch(FORWARD_PIN): Toggles        *
*         through different grip modes              *
*                                                   *
* Outputs: -Motor activation(step_pins):            *
*          Activates the stepper motors             *
*          -Motor Direction(dir_pins):              *
*          Sets the motor direction                 *
*                                                   *
****************************************************/



#define NUMPINS 3
#define POT A3 //CoolEN
#define FORCE_ONE A0 //Abort
#define FORCE_TWO A1 //Hold
#define FORCE_THREE A2 //Resume
#define FORWARD_PIN 12 //SpnEn
#define EMG_PIN A4 //X-
#define EMG_MODE_PIN 9 //SCL


#define MOTOR_MAX_T 150
#define MOTOR_MAX_1 330
#define MOTOR_MAX_23 330 

#define MOTOR_MIN 0
#define NUM_STATES 4
#define POT_CAL 0.7
#define EMG_THRESH 500
#define HIGH_THRESH 500



const int step_pins[NUMPINS] = {2,3,4};
const int dir_pins[NUMPINS] = {5,6,7}; 

//Initialise Variables
int open_close = 1; 
int active_1 = 0;
int active_2 = 0;
int active_3 = 0;
int step1 = 0;
int step2 = 0;
int step3 = 0;
int idle = 1;
int hand_mode = 0;
int EMG_state = 0;
int EMG_mode = 0;
int toggle = 0;
int toggle_gesture = 1;
int pot = 1;



void setup() {

  //Initialise pins and start serial for monitoring
  Serial.begin(9600);
  pinMode(FORWARD_PIN,INPUT);
  pinMode(EMG_PIN,INPUT);
  
  
  for(int i = 0;i < NUMPINS; i++){
    pinMode(dir_pins[i],OUTPUT); 
    pinMode(step_pins[i],OUTPUT);
  }

  

  
}

void loop() {

  //Idle Mode, Hand is waiting for EMG input with open palm
  if(idle){
 
    getEMG();
    getHandMode();
    getEMGMode();

    //Set idle state to be off if toggle is 1, toggle ensures hand is idle
    //before being activated again 
    if(!EMG_state){
        toggle  = 1;
    }else if(EMG_state&&toggle){
        idle = 0;
        toggle = 0;
    }
    
    open_close  = 1;

  //Hand opening mode when idle = 0 and hand is ready to be opened (open_close = 1)
  }else if(open_close){
    
    //Gets the EMG data and chooses on/off state
    if(EMG_mode){
      //Continuous mode, open_close will return to opening(0) when EMG is no longer
      //activated
      getEMG();
      open_close = EMG_state;
    }else{
      //toggle mode. EMG signal must go from high to low to high to change activation
      getEMG();
      if(!EMG_state){
        toggle  = 1;
      }else if(EMG_state&&toggle){
        open_close = 0;
        toggle = 0;
      }
    }
    getDir();
    
    setMode();
    
    motorStepFor();

  // Opening hand, returns to rest position  
  }else{
    getDir();
    motorStepBack();
    
    //Tests if motors have reached rest position
    if(active_1||active_2||active_3){
      idle = 0; 
    }else{
      idle = 1;
      delayMicroseconds(500);
    }
  }
}


//Funtion Definitions//

//Sets the hand mode
void setMode(void){

  //Mode 1: Full close. Every finger closes maximum amount until threshold  
    if(hand_mode == 0){  
      active_1 = (step1 < MOTOR_MAX_T); 
      active_2 = (step2 < MOTOR_MAX_1); 
      active_3 = (step3 < MOTOR_MAX_23); 
      
  //Mode 2: Pinch Grip. Only thumb and first finger close  
    }else if(hand_mode == 1){
      active_1 = (step1 < MOTOR_MAX_T); 
      active_2 = (step2 < MOTOR_MAX_1); ;
      active_3 = 0;

  //Mode 3: Hammer Grip. Closes the fingers first, then thumb. Good for grabbing 
  //thinner objects      
    }else if(hand_mode == 2){
      if(step2 > 0.9* MOTOR_MAX_1){
        active_1 = (step1 < MOTOR_MAX_T);
      } 
      active_2 = (step2 < MOTOR_MAX_1); ;
      active_3 = (step3 < MOTOR_MAX_23);
    }

  //Mode 4: Bowel grip. Fingers engage half way. Good for carrying things with 
  //a somewhat open palm  
    if(hand_mode == 3){  
      active_1 = (step1 < (MOTOR_MAX_T/2)); 
      active_2 = (step2 < (MOTOR_MAX_1/2)); 
      active_3 = (step3 < (MOTOR_MAX_23/2)); 
    }
}
    
//Sets the direction pins
void getDir(void){  
  if(open_close){
    for(int i = 0;i < NUMPINS; i++){
    digitalWrite(dir_pins[i],HIGH);
  }
  }else{
    for(int i = 0;i < NUMPINS; i++){
    digitalWrite(dir_pins[i],LOW);
    }
  }
      
}

//Steps the motors forward to close the hand
void motorStepFor(void){

  //Compares force from force resistive sensors on fingers to the 
  //maximum threshhold set by the potentiometer
  compareForce();
  
  if(active_1){
    digitalWrite(step_pins[0],HIGH);
    step1++;
  }
  if(active_2){
    digitalWrite(step_pins[1],HIGH);
    step2++;
  }
  if(active_3){
    digitalWrite(step_pins[2],HIGH);
    step3++;
  }
  delayMicroseconds(700);
  digitalWrite(step_pins[0],LOW);
  digitalWrite(step_pins[1],LOW);
  digitalWrite(step_pins[2],LOW);
  delayMicroseconds(700);    
}

//Steps the motors backward to open the hand
void motorStepBack(void){
  active_1 = (step1 > MOTOR_MIN); 
  active_2 = (step2 > MOTOR_MIN); 
  active_3 = (step3 > MOTOR_MIN);
  
  if(active_1){
    digitalWrite(step_pins[0],HIGH);
    step1--;
  }
  if(active_2){
    digitalWrite(step_pins[1],HIGH);
    step2--;
  }
  if(active_3){
    digitalWrite(step_pins[2],HIGH);
    step3--;
    
  }
  delayMicroseconds(700);
  digitalWrite(step_pins[0],LOW);
  digitalWrite(step_pins[1],LOW);
  digitalWrite(step_pins[2],LOW);
  delayMicroseconds(700);    
}


//Compares the force between the force resistive sensors in the fingertips to
//the maximum force set by the potentiometer and sets active = 0 if 
//threshold met
void compareForce(void){

  pot = analogRead(POT)*POT_CAL;

  if(analogRead(FORCE_ONE)>pot){
    active_1 = 0;
  }
  if(analogRead(FORCE_TWO)>pot){
    active_2 = 0;
  }
  if(analogRead(FORCE_THREE)>pot){
    active_3 = 0;
  }
  
}


void getHandMode(void){
   
   int bState_next =digitalRead(FORWARD_PIN);
   
 
  if((bState_next == HIGH)&&toggle_gesture){
      hand_mode = (hand_mode + 1)%NUM_STATES ;;
      toggle_gesture = 0;
   }else if(bState_next == LOW){
    toggle_gesture = 1;
   }
   
}

void getEMG(void){
  int EMG_state = digitalRead(EMG_PIN);

  //Checks if EMG is above the threshold (EMG is HIGH)
  if (analogRead(EMG_PIN) > EMG_THRESH){
      EMG_state  = 1;
  }else{
      EMG_state  = 0;
  }
}

//Gets the EMG mode from a digital input from a switch
void getEMGMode(void){
 int EMG_val = analogRead(EMG_MODE_PIN);
  if (EMG_val == HIGH ){
      EMG_mode = 1 ;
   }else {
      EMG_mode= 0;
   }
}



   
