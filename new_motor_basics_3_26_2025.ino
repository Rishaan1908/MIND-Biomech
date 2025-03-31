// Define pin connections & motor's steps per revolution
const int dirPin_thumb = 8;
const int stepPin_thumb = 9;
const int dirPin_point = 2;
const int stepPin_point = 3;
const int dirPin_fingers = 4;
const int stepPin_fingers = 5;
const int stepsPerRevolution = 1000;
int stepDelay=1000;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin_thumb, OUTPUT);
  pinMode(dirPin_thumb, OUTPUT);
  pinMode(stepPin_point, OUTPUT);
  pinMode(dirPin_point, OUTPUT);
  pinMode(stepPin_fingers, OUTPUT);
  pinMode(dirPin_fingers, OUTPUT);
}
void loop()
{
  tighten(true, true, true);
  delay(1000);
  stop();
  delay(500);
}

void tighten(bool thumb, bool point, bool fingers){
  // Set direction
  digitalWrite(dirPin_thumb, LOW);
  digitalWrite(dirPin_fingers, HIGH);
  digitalWrite(dirPin_point, LOW); 
  // Spin motor
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    if(thumb){
      digitalWrite(stepPin_thumb, HIGH);
    }
    if(point){
      digitalWrite(stepPin_point, HIGH);     
    }
    if(fingers){
      digitalWrite(stepPin_fingers, HIGH);      
    }
    delayMicroseconds(stepDelay);
    if(thumb){
      digitalWrite(stepPin_thumb, LOW);
    }
    if(point){
      digitalWrite(stepPin_point, LOW);     
    }
    if(fingers){
      digitalWrite(stepPin_fingers, LOW);      
    }
    delayMicroseconds(stepDelay);
  }
}

void loosen(bool thumb, bool point, bool fingers){
  // Set direction
  digitalWrite(dirPin_thumb, HIGH);
  digitalWrite(dirPin_fingers, LOW);
  digitalWrite(dirPin_point, HIGH); 
  // Spin motor
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    if(thumb){
      digitalWrite(stepPin_thumb, HIGH);
    }
    if(point){
      digitalWrite(stepPin_point, HIGH);     
    }
    if(fingers){
      digitalWrite(stepPin_fingers, HIGH);      
    }
    delayMicroseconds(stepDelay);
    if(thumb){
      digitalWrite(stepPin_thumb, LOW);
    }
    if(point){
      digitalWrite(stepPin_point, LOW);     
    }
    if(fingers){
      digitalWrite(stepPin_fingers, LOW);      
    }
    delayMicroseconds(stepDelay);
  }
}

void stop(){
  digitalWrite(stepPin_thumb, LOW);
  digitalWrite(stepPin_point, LOW);
  digitalWrite(stepPin_fingers, LOW);
}
