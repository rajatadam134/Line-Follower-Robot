// - sensor placement is shown below 7 channel analog sensor array..
// - sensor values :- black(1000) and white(less than 100) 
//    _______       _______       _______       _______       _______        _______       _______
//   |__0_0__|     |__0_0__|     |__0_0__|     |__0_0__|     |__0_0__|      |__0_0__|     |__0_0__|
//   Sensor[1]     Sensor[2]     Sensor[3]     Sensor[4]     Sensor[5]      Sensor[6]     Sensor[7]
//    (Left)                                   (center)                                    (Right)
//                                   



#include <SparkFun_TB6612.h> // Library for Motor Deiver..
#include<SoftwareSerial.h>  // Library for Bluetooth Communication..
SoftwareSerial B(0,1);     //RX,TX

#define AIN1 5  // checked        -
#define BIN1 7  // checked       |
#define AIN2 4  // checked       | ----> Motor Driver connection to Controller
#define BIN2 8  // checked       |
#define PWMA 3  // checked       |
#define PWMB 9  // checked       |
#define STBY 6  // checked        -


const int offsetA = 1; // Motor 'A' OFFSet Set HIGH.. 
const int offsetB = 1; // Motor 'B' OFFSet Set HIGH..

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int P, D, I, previousError, PIDvalue, error, activeSensors; // Global Controle Variables..
int LeftSpeed, RightSpeed; // Global Speed Variables..
String Path;  // Path Datatype *[Where our Path to be Stored]..
int J=0;

int sensorWeight[5] = { 4, 1, 0, -1, -4 };
int MotorSpeed = 70;   // motor Speed.
int turnspeed = 39; // bot turning speed..
int pdSpeed = 20; // PID Correction Speed..
int Forward_Delay = 500; // Node Check Delay *[Move Forward to check Condition]


int botcalibrate_button = 0;
int DryRun_button = 0;
int ActualRun_button =0;


float Kp = 0;         // Proptional Constant decleration..
float Kd = 0;         // Derivative Constant decleration..
float Ki = 0;         // Integral Constant decleration..


int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];  // Motor Array numbers [minValue] , [maxValues] , [threshold]


void setup() {
  
  B.begin(9600);  // Bluetooth Baud Rate (9600)..
  
  pinMode(12, INPUT_PULLUP);         //  DRY RUN SWITCH..
  pinMode(10, INPUT_PULLUP);         //  Actual RUN SWITCH..
  pinMode(11, INPUT_PULLUP);         //  Calibrate SWITCH..
  pinMode(13,OUTPUT);                //  LED indicator..

}
void loop() 
{  

/*..........................................  Calibration Loop   ........................................................................................................................................................................*/
    
   
    // int Calibrate_input_button = digitalRead(11);
    // delay(100);
        //  if(botcalibrate_button != Calibrate_input_button)     // Reading the state of button..
        //  {          
            // botcalibrate_button = Calibrate_input_button;
            // if(botcalibrate_button == 0)                      // comparing with state. (without press 1 , and when press 0)
            // {               
               while (digitalRead(11)) {}
               delay(1000);
               B.print("Calibration Started");
               B.println(" ");
               calibrate();
               B.print("Calibrated");
               B.println("  ");
            // } 
        //  } 


      



/*..........................................  DryRun Loop   ........................................................................................................................................................................*/


  // int DryRun_input_button = digitalRead(12);
  // delay(100);
  //    if(DryRun_button != DryRun_input_button)
  //    {
  //      DryRun_button = DryRun_input_button;
  //    }
  //      if(DryRun_button == 0)            // for dry run triversing the maze and record path
      //  {   
         while (digitalRead(12)) {}
         delay(1000);                       
         B.print("Dry Run Started");
         B.println(" ");
         while(1)
         {

           if((analogRead(0) < threshold[0]) && (analogRead(4) > threshold[4]))
           {
            botTurnStop();
            botinchforward();
            botTurnStop();
             if(analogRead(2) > threshold[2])
             {
               
              //  if(analogRead(3) > threshold[3] && analogRead(4) > threshold[4] )
              //  {
              //    EndOfMaze();
              //    Path+='E';
              //    B.print("E");    //                 |                 _______
              //    B.print(";");    //            _____|_____     or    |__END__|      go little forward and check if all sensors are white then END          
              //    B.print(Path);   //                 |                    |
              //    B.print(";");    //                 |                    |
              //    Path = shortpath(Path);
              //    B.print(Path);  
              //    B.print(";");
              //    break;
              //  }
                 Path+='S';
                 B.print("S");
                 B.print(";");      //                   |                for right and straight intersection
                 botTurnStop();     //                   |______          check if straight present else go right
                 PID();             //                   |                consider left as priority and the straight
            }                       //                   |                     
            else{             
            Path+='R';
            botRight();
            botTurnStop();
            B.print("R");
            B.print(";");
           }
         }
    else if((analogRead(0) > threshold[0]) && (analogRead(4) < threshold[4]))
    { 
      botTurnStop();              
      botinchforward();
      botTurnStop();
      // if(analogRead(0) > threshold[0] && analogRead(1) > threshold[1])
      // {
      //  EndOfMaze();
      //  Path+='E';         //                |                 _______
      //  B.print("E");     //            _____|_____     or    |__END__|      go little forward and check if all sensors are white then END   
      //  B.print(";");    //                  |                    |
      //  B.print(Path);  //                   |                    |
      //  B.print(";");         
      //  Path = shortpath(Path);
      //  B.print(Path);
      //  B.print(";");
      //  break;
      // }                  
      botLeft();          //           |                     
      Path+='L';         //        ____|          or        ____    Go left
      B.print("L");     //             |                       |  
      B.print(";");    //              |                       |
      botTurnStop();
    }
    else if ((analogRead(0) > threshold[0]) && (analogRead(1) > threshold[1]) && (analogRead(2) > threshold[2] )&& (analogRead(3) > threshold[3]) && (analogRead(4) > threshold[4]))
    {
      botTurnStop();
      botinchforward();
      botTurnStop();
      // if((analogRead(1) > threshold[1] && analogRead(0) > threshold[0]) || (analogRead(3) > threshold[3] && analogRead(4) > threshold[4]))
      // {
      //  EndOfMaze();
      //  Path+='E';
      //  B.print("E");
      //  B.print(";");
      //  B.print(Path);
      //  B.print(";");
      //  Path = shortpath(Path);
      //  B.print(Path);           //                |                 _______
      //  B.print(";");           //            _____|_____     or    |__END__|      go little forward and check if all sensors are white then END 
      // break;                  //                  |                    |           else go left
      // }                      //                   |                    |
      // else{
      Path+='L';
      B.print("L");
      B.print(";");
      botTurnStop();
      botLeft();
      botTurnStop();
    // }
   }
    else if ((analogRead(0) < threshold[0]) && (analogRead(1) < threshold[1]) && (analogRead(2) < threshold[2]) && (analogRead(3) < threshold[3]) && (analogRead(4) < threshold[4] ))
    {  
      Path+='U';
      B.print("U");
      B.print(";");                      
      botTurnStop() ;   
      botuturn ();//   if deadend found then take Uturn..
      botTurnStop() ;                          
    }
     if (analogRead(2) >= threshold[2])
    {
      B.println("PID");
      PID ();  // if center senser is white then follow PID routine..
    }
   }
  // }

/*..........................................  ActualRun Loop   ........................................................................................................................................................................*/

    // int ActualRun_input_button = digitalRead(10);
    // delay(100);
    //     if(ActualRun_button != ActualRun_input_button)
    //     {
    //       ActualRun_button = ActualRun_input_button;
    //     }
    //     if(ActualRun_button == 0)
    //     {

           while (digitalRead(10)) {}
           delay(1000);
           B.print(Path);
           unsigned int i=0;
           while(J==0)
           {
              if(analogRead(2) > threshold[2])
              {
                 PID();
              }
              if(analogRead(4) < threshold[4] && analogRead(0) > threshold[0])     // left Path Check..
              {
                 botTurnStop();
                 botinchforward();
                 if(analogRead(2) > threshold[2])        // Straight Path Check..
                 {
                    if( analogRead(0) > threshold[0] && analogRead(1) > threshold[1]) // End Condition Check..
                    {
                       if(Path.charAt(i)=='E')
                       {
                         EndOfMaze();
                         J=1;
                       }
                    }
                       if(Path.charAt(i)=='S')
                       {
                          PID();
                          i++;
                       }
                   }
              
                else
                {
                   botLeft();
                   botTurnStop();
                   i++;
                }
              }
   else if(analogRead(4) > threshold[4] && analogRead(0) < threshold[0])     // right  check
   {
        botTurnStop();
        botinchforward(); 
        if( analogRead(4) < threshold[4])  // Straight Condition Check
        {   
            if(analogRead(3) > threshold[3] && analogRead(4) > threshold[4])  // End Condition Check
            {
               if(Path.charAt(i)=='E')
               {
                 EndOfMaze();
                 J=1;
               }
            }
            if(Path.charAt(i)=='S')
            {
              PID();
              i++;
            }
        }
        else
        {
           botRight();
           botTurnStop();
           i++;
         }
       }
     }
  //  }
 }
 


String shortpath(String Path)
{      
   for(int P=0; P<=10; P++)
   { 
     Path.replace("LUL","S");             
     Path.replace("LUS","R");  
     Path.replace("RUL","U");         // path optimizing techniques
     Path.replace("SUS","U");
     Path.replace("SUL","R");
     Path.replace("LUR","U");
   }
   return Path;
}


void PID() 
{
      // Kp = 0.00006 * (analogRead(2));   // proptional error
      // Kd = 50 * Kp;                            // derivative error
      // Ki = 0.0001;                             // Integral error
      Kp = 0.08;
      Kd = 0.15;
      Ki = 0;
      linefollow();
}


void linefollow()
{
  error = 0;
  activeSensors = 0;

  // int error = (((analogRead(2) + analogRead(3))/2) - ((analogRead(6) + analogRead(5))/2));    // error is calculated between sensor 3 and 5
  // int error = (analogRead(3) - analogRead(1));

  for (int i = 0; i < 5; i++) {
    // sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    B.print(sensorValue[i]);
    B.print("   ");
  }
  B.println("  ");
  // int error = (3 * sensorValue[0] + sensorValue[1] - sensorValue[3] - 3 * sensorValue[4]);
  // error = error * -1;

    for (int i = 0; i < 5; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;

      P = error;
      I = I + error;
      D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  LeftSpeed = MotorSpeed - PIDvalue;
  RightSpeed = MotorSpeed + PIDvalue;

  if (LeftSpeed > 255) 
  {
    LeftSpeed = pdSpeed;
  }
  if (LeftSpeed < 0) 
  {
    LeftSpeed = 0;
  }
  if (RightSpeed > 255) 
  {
    RightSpeed = pdSpeed;
  }
  if (RightSpeed < 0) 
  {
    RightSpeed = 0;
  }
  
  motor1.drive(LeftSpeed);
  motor2.drive(RightSpeed);
  
}


void calibrate()
{
  for ( int i = 0; i < 5; i++)              //calebration loop
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 10000; i++)           // this loop runs 2000 times and stores different data from each sensor
  {
    digitalWrite(13 , HIGH);
    motor1.drive(180);
    motor2.drive(-180);

    for ( int i = 0; i < 5; i++)          // this loop is to calculate maximum and minimum value for each sensor
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 0; i < 5; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;    // Threshold = (maxValue + minValue) / 2
    B.print(threshold[i]);
    B.print("  ");
  }
   B.println(" ");
   digitalWrite(13 , LOW);
   motor1.drive(0);
   motor2.drive(0); 
}



void botuturn () 
{
  B.println("U-TURN");
  motor1.drive(-1*turnspeed);     // Uturn Function..
  motor2.drive(turnspeed);
  delay(100);
  while (analogRead(2) < threshold[2])
  {
    motor1.drive(-1*turnspeed);
    motor2.drive(turnspeed);
  }
}



void botinchforward() 
{           
  B.println("INCH-FORWARD");
  motor1.drive(100);                 // move inch forward for reposition to center and find if there is path 
  motor2.drive(100);                //  present to frount or not..
  delay(Forward_Delay);
}


void botRight() 
{
  B.println("RIGHT");
  motor1.drive(-1*turnspeed);
  motor2.drive(turnspeed);
  delay(180);
  while (analogRead(2) < threshold[2])     
  {
    motor1.drive(-1*turnspeed);
    motor2.drive(turnspeed);
  }
}


void botLeft() 
{
  B.println("LEFT");
  motor1.drive(turnspeed);
  motor2.drive(-1*turnspeed);
  delay(180);
  while (analogRead(2) < threshold[2])   // Left Turn Function..
  {         
    motor1.drive(turnspeed);
    motor2.drive(-1 * turnspeed);
  }
 }

 
void botTurnStop() 
{
  B.println("STOP");
  motor1.drive(0);
  motor2.drive(0);
  delay(20);
}
void EndOfMaze()
{
  B.println("END");
  motor1.drive(0);
  motor2.drive(0);
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
}


