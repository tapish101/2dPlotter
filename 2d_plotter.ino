/*
Arduino bassed 2d pen plotter using dvd drivers
myStepperX is x axix stepper conncted at pin 4,5,7 and 7. mySteppery for y axis on digital pins 8,9,10
and 11. Each pairs of wire(like 4 and 5, 8 and 9) can be altered for cange in dir of stepper motor.

Importantly pay close attention on wiring diagram.
 */



//L293d enable pin
const int L293dEnable= A0;
const int led= A1; // led to show driver state

#include <Servo.h>
#include <Stepper.h>

#define LINE_BUFFER_LENGTH 512

// servo pen up and down angle
const int penZUp = 35;
const int penZDown = 91;

// Servo no Pino 10
const int penServoPin = 3;

// step per revolution for stepper motor (usualiy 20 for dvd motors)
const int stepsPerRevolution = 20; 
//to flop enable pin high and low( false mean low)
bool state=false;

Servo penServo;  

// defining X and Y axis servos
Stepper myStepperX(stepsPerRevolution, 5,4,6,7);            
Stepper myStepperY(stepsPerRevolution, 9,8,10,11);  

/* global variables   */
struct point { 
  float x; 
  float y; 
  float z; 
};

struct point actuatorPos;

// drawing settings  
float StepInc = 1;
int StepDelay = 0;
int LineDelay = 50;
int penDelay = 50;

// motor steps to walk 1 mm
float StepsPerMillimeterX = 4.54;
float StepsPerMillimeterY = 4.54;

// writing area
float Xmin = 0;
float Xmax = 38;
float Ymin = 0;
float Ymax = 38;
float Zmin = 0;
float Zmax = 1;

float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmax; 

// Configurar true para depurar saída
boolean verbose = false;


void setup() {

  Serial.begin( 9600 );
  // x axis limit switch input
  pinMode(13,INPUT);
  
  //y axis limit switch input
  pinMode(12,INPUT);
  
  pinMode(L293dEnable,OUTPUT);
  digitalWrite(L293dEnable,LOW);//default disable l293d 
  digitalWrite(led,LOW);
  
  
  /*interrrupt triggers L293D enable pin high/low
   * Has to take care because of overheating of driver ICs 
   * tried to lower corrent but power of stepper drop significantly
   * so I put custom made (very beautiful) heatsing on ICs and enable then only when i need Draw
   */
  attachInterrupt(0,ENABLE,RISING);

  // setting max speed for stepper motors
  myStepperX.setSpeed(250);
  myStepperY.setSpeed(250);  

  Serial.print("X goes from "); 
  Serial.print(Xmin); 
  Serial.print(" to "); 
  Serial.print(Xmax); 
  Serial.println(" mm."); 
  Serial.print("Y goes from "); 
  Serial.print(Ymin); 
  Serial.print(" to "); 
  Serial.print(Ymax); 
  Serial.println(" mm."); 
}

void loop() 
{

  
  delay(200);
  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (1) {

    // Recepção serial
    while ( Serial.available()>0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {       
        if ( lineIndex > 0 ) {                        
          line[ lineIndex ] = '\0';                   
          if (verbose) { 
            Serial.print( " Received: "); 
            Serial.println( line ); 
          }
          processIncomingLine( line, lineIndex );
          lineIndex = 0;
        } 
        else { 
          //Empty comment line, skip block.  
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");    
      } 
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   
          if ( c == ')' )  lineIsComment = false;     
        } 
        else {
          if ( c <= ' ' ) {                           
          } 
          else if ( c == '/' ) {                    
          } 
          else if ( c == '(' ) {                    
            lineIsComment = true;
          } 
          else if ( c == ';' ) {
            lineSemiColon = true;
          } 
          else if ( lineIndex >= LINE_BUFFER_LENGTH-1 ) {
            Serial.println( "ERROR - buffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          } 
          else if ( c >= 'a' && c <= 'z' ) {        
            line[ lineIndex++ ] = c-'a'+'A';
          } 
          else {
            line[ lineIndex++ ] = c;
          }
        }
      }
    }
  }
}

void processIncomingLine( char* line, int charNB ) {
  int currentIndex = 0;
  char buffer[ 64 ];                               
  struct point newPos;

  newPos.x = 0.0;
  newPos.y = 0.0;
  
  while( currentIndex < charNB ) {
    switch ( line[ currentIndex++ ] ) {             
    case 'U':
      penUp(); 
      break;
    case 'D':
      penDown(); 
      break;
    case 'G':
      buffer[0] = line[ currentIndex++ ];         
      buffer[1] = '\0';

      switch ( atoi( buffer ) ){                   // working with g command
      case 0:                                   //no need to diff b/w move or move faster 
      case 9:
      Homing();
      break;
      case 1:
        // imp:: Here we have assume that the gcode string received have X vaiue before Y
        char* indexX = strchr( line+currentIndex, 'X' ); 
        char* indexY = strchr( line+currentIndex, 'Y' );
        if ( indexY <= 0 ) {
          newPos.x = atof( indexX + 1); 
          newPos.y = actuatorPos.y;
        } 
        else if ( indexX <= 0 ) {
          newPos.y = atof( indexY + 1);
          newPos.x = actuatorPos.x;
        } 
        else {
          newPos.y = atof( indexY + 1);
          indexY = '\0';
          newPos.x = atof( indexX + 1);
        }
        drawLine(newPos.x, newPos.y );
   
        actuatorPos.x = newPos.x;
        actuatorPos.y = newPos.y;
        break;
      }
      break;
    case 'M':
      buffer[0] = line[ currentIndex++ ];        // /!\ M commands must not increase 3-digit
      buffer[1] = line[ currentIndex++ ];
      buffer[2] = line[ currentIndex++ ];
      buffer[3] = '\0';
      switch ( atoi( buffer ) ){
      case 300:
        {
          char* indexS = strchr( line+currentIndex, 'S' );
          float Spos = atof( indexS + 1);
          if (Spos == 30) { 
            penDown(); 
          }
          if (Spos == 50) { 
            penUp(); 
          }
          break;
        }
      case 114:                                //report position 
        Serial.print( "Absolute position : X = " );
        Serial.print( actuatorPos.x );
        Serial.print( "  -  Y = " );
        Serial.println( actuatorPos.y );
        break;
        
      case 17:              //enabling drivers for stepper motor anf attaching servo                
        digitalWrite(L293dEnable,HIGH);
        digitalWrite(led,HIGH);
        penServo.attach(penServoPin);
        state=true;
        break;

        case 18:         //disabling drivers                     
        digitalWrite(L293dEnable,LOW);
        digitalWrite(led,LOW);
        penServo.detach();
        state=false;
        break;
        
      default:
        Serial.print( "Unrecognnized M command");
        Serial.println( buffer );
      }
    }
  }



}


/********************************* 
 * int (x1;y1) : initial coordinates
 * int (x2;y2) : final coordinates
 **********************************/
void drawLine(float x1, float y1) {

  if (verbose)
  {
    Serial.print("fx1, fy1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }  

  // setting limits for each axis
  if (x1 >= Xmax) { 
    x1 = Xmax; 
  }
  if (x1 <= Xmin) { 
    x1 = Xmin; 
  }
  if (y1 >= Ymax) { 
    y1 = Ymax; 
  }
  if (y1 <= Ymin) { 
    y1 = Ymin; 
  }

  if (verbose)
  {
    Serial.print("Xpos, Ypos: ");
    Serial.print(Xpos);
    Serial.print(",");
    Serial.print(Ypos);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("x1, y1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }

  //converting coordinates to steps  
  x1 = (int)(x1*StepsPerMillimeterX);
  y1 = (int)(y1*StepsPerMillimeterY);
  float x0 = Xpos;
  float y0 = Ypos;

  // finding direction of cordinates
  long dx = abs(x1-x0);
  long dy = abs(y1-y0);
  int sx = x0<x1 ? StepInc : -StepInc;
  int sy = y0<y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i=0; i<dx; ++i) {
      myStepperX.step(sx);
      over+=dy;
      if (over>=dx) {
        over-=dx;
        myStepperY.step(sy);
      }
      delay(StepDelay);
    }
  }
  else {
    for (i=0; i<dy; ++i) {
      myStepperY.step(sy);
      over+=dx;
      if (over>=dy) {
        over-=dy;
        myStepperX.step(sx);
      }
      delay(StepDelay);
    }    
  }

  if (verbose)
  {
    Serial.print("dx, dy:");
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("Going to(");
    Serial.print(x0);
    Serial.print(",");
    Serial.print(y0);
    Serial.println(")");
  }

  //delay b/w next line
  delay(LineDelay);
  //updating positions
  Xpos = x1;
  Ypos = y1;
}


void penUp() { 
  penServo.write(penZUp); 
  delay(LineDelay); 
  Zpos=Zmax; 
  if (verbose) { 
    Serial.println("pen up"); 
  } 
}


void penDown() { 
  penServo.write(penZDown); 
  delay(LineDelay); 
  Zpos=Zmin; 
  if (verbose) { 
    Serial.println("pen down"); 
  } 
  delay(300);
}

//very basic homing seq but if it works its perfect..:)
void Homing(){
  //enabling drivers
  digitalWrite(A0,HIGH);
  digitalWrite(A1,HIGH);
  penServo.attach(penServoPin);
  penServo.write(penZUp);
  delay(100);
  //x axis homing
  while(digitalRead(13)==LOW){
    myStepperX.step(-1);
  }
  //y axis homing
  while(digitalRead(12)==LOW){
    myStepperY.step(-1);
  }
  //disabling drivers
  digitalWrite(A0,LOW);
  digitalWrite(A1,LOW);
  penServo.detach();
}

/*
 * ENABLE() function changes the current state of the drivers
 * using it with emergency stop switch
 */
void ENABLE(){
  //millies() for debouncing of switch
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
    if(state) {
      digitalWrite(L293dEnable,LOW);
      digitalWrite(led,LOW);
      penServo.detach();
      state=false;
    }
    else{
      digitalWrite(L293dEnable,HIGH);
      digitalWrite(led,HIGH);
      penServo.attach(penServoPin);
      penServo.write(penZUp);
      delay(80);
      state=true;
    }
  }
  last_interrupt_time = interrupt_time;
}
