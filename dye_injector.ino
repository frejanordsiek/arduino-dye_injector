/* Copyright 2013 Freja Nordsiek
   
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
   
       http://www.apache.org/licenses/LICENSE-2.0
   
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

/* Dye Injector
   
   Controller for a dye injector using a stepper motor to control
   the position of a syringe pump. Uses the Adafruit Motor Shield
   V2 (http://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino).
   
   Author:   Freja Nordsiek
   Notes:
   History:  * 2013-09-21 Created.
*/

/* There are four different commands that are accepted over the
   serial link. An invalid command causes a response of "Invalid".
   All serial comminication are done at 115200 bps and are
   newline terminated.
   
   Get Program Status:
     "Status?"
     
     Request the status of this program. Responds "OK" if the
     program is working right.
   
   Get Program Version:
     "Version?"
     
     Requests the version of this program. Responds with the
     version string.
   
   Get Program Name:
     "Program?"
     
     Requests the name of this program. Responds with
     "Dye Injector".
   
   Get the Number of Steps per Revolution:
     "StepsPerRevolution?"
     
     Returns the number of steps the stepper motor makes per
     revolution.
   
   Get Volume Injected per Revolution:
     "VolumePerRevolution?"
     
     Returns the volume of dye (in mL) injected per revolution of
     the stepper motor as a floating point number.
   
   Get Volume of Dye Injected Since Program Started:
     "DyeInjected?"
     
     Returns the volume of dye (in mL) injected by this program
     since it started as a float.
     
   Reset Dye Injected Volume:
     "ResetDyeInjected"
     
     Resets the count of the volume of dye injected so far back
     to zero. Returns "ACK".
     
   Inject Dye:
     "Inject: A B,C D,E F"
     
     Command to inject dye. Injection is specified by stages that
     are comma separated. The maximum number of stages currently
     supported is 4. Each stage consists of a number of steps to
     rotate the stepper (+ is inject and - is suck) and the
     speed in RPM to rotate the stepper (must be a positive
     integer) separated by a space. In the above example, there
     are three stages with A,  C, and E as the number of steps
     and B, D, and F as the speeds.
   
*/

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Version of the program.

const String freqdriveMotorControlVersion = "0.1";

/* A string is needed to hold serial commands sent by the computer
   along with a flag to indicate whether the command is complete
   or not (not complete till it is terminated with a newline).
   This scheme is done so that the program is still functioning
   while waiting for commands from the computer. Also, the max
   length of the command string needs to be known.
*/

String commandFromComputerString = "";
boolean commandFromComputerComplete = false;
int commandFromComputerMaxLength = 256;

// Need to know the number of positions per revolution of
// the stepper motor as well as the volume (in mL) of dye injected
// per revolution.

const unsigned int stepsPerRevolution = 200;
const float volumePerRevolution = (60.0/4.2188)/40.0;

// Need to keep track of the amount of dye (in mL) injected since
// the arduino has been turned on.

float dyeInjected = 0;

// Arrays to keep the number of steps (sign indicates direction)
// and stepper speeds for each stage of an injection command. The
// maximum number of stages allowed also needs to be specified.

#define MAX_INJECTION_STAGES 4
unsigned int injectionStageSpeeds[MAX_INJECTION_STAGES];
int injectionStageSteps[MAX_INJECTION_STAGES];

// Create the motor shield (default address as there is only
// one) and the stepper motor (connected to channel 1 and has
// 200 positions per revolution).
  
Adafruit_MotorShield MotorShield = Adafruit_MotorShield(); 
Adafruit_StepperMotor *stepper = MotorShield.getStepper(stepsPerRevolution, 1);


void setup()
{
  
  // Reserve sufficient space in the motor command string for
  // commands.
  
  commandFromComputerString.reserve(commandFromComputerMaxLength);
  
  // We want to communicate with the computer at the fastest speed
  // possible.
  
  Serial.begin(115200);
  
  // The start the shield (default PWM frequency).
  
  MotorShield.begin();
  
}

void loop()
{
  
  // If we have a complete command from the serial, it needs to be
  // processed.
  
  if (commandFromComputerComplete)
    {
    
      // Trim off the newline and any characters after it (will
      // only deal with one command at a time).
      
      commandFromComputerString = commandFromComputerString.substring(0,commandFromComputerString.indexOf('\n'));
      
      // Check the command string for each of the 11 commands in
      // turn, do the appropriate command, or respond with "Invalid"
      // if it was not a valid command.
      
      if (commandFromComputerString == "Status?")
        Serial.print("OK\n");
      else if (commandFromComputerString == "Program?")
        Serial.print("Dye Injector\n");
      else if (commandFromComputerString == "Version?")
        Serial.print(freqdriveMotorControlVersion + "\n");
      else if (commandFromComputerString == "DyeInjected?")
        Serial.print(floatToString(dyeInjected) + "\n");
      else if (commandFromComputerString == "ResetDyeInjected")
        {
          dyeInjected = 0;
          Serial.print("ACK\n");
        }
      else if (commandFromComputerString == "StepsPerRevolution?")
        Serial.print(String(stepsPerRevolution) + "\n");
      else if (commandFromComputerString == "VolumePerRevolution?")
        Serial.print(floatToString(volumePerRevolution) + "\n");
      else if (commandFromComputerString.startsWith("Inject: "))
        {
          // Hand off the command processing to a function that
          // reads all the stages and returns the number that will
          // be done, which will be set to -1 if there is something
          // invalid with the command.
          
          int numberStages = processInjectCommand(commandFromComputerString);
          
          if (numberStages == -1)
            Serial.print("Invalid\n");
          else
            {
              // The command is good, so now we execute it.
              
              Serial.print("Executing\n");
              
              // Do each stage. Update the amount of dyeInjected
              // each time.
              
              for (int i = 0; i < numberStages; i++)
                {
                  stepper->setSpeed(injectionStageSpeeds[i]);
                  stepper->step((unsigned int)abs(injectionStageSteps[i]),
                                injectionStageSteps[i] > 0 ? FORWARD : BACKWARD,
                                DOUBLE);
                  stepper->release();
                  dyeInjected += (float)injectionStageSteps[i]
                                 * volumePerRevolution
                                 / (float)stepsPerRevolution;
                }
            }
        }
      else // As it wasn't a recognized command, return "Invalid".
        Serial.print("Invalid\n");
    
    // Clear the command string and reset the flag saying that
    // the command is complete (that a new command is waiting to
    // be processed).
    
    commandFromComputerString = "";
    commandFromComputerComplete = false;
    
  }
  
}

/* This is basically copied from the SerialEvent example with just
   formatting and naming changes.
   
   This is called after each iteration of loop if there is any new
   data on the serial RX line. All available bytes are grabbed and
   appended onto commandFromComputerString, which is marked as
   complete if a newline character was received.
*/
void serialEvent()
{
  
  // Grab each and every byte available.
  
  while (Serial.available())
    {
      
      // Read the first available char.
      
      char currentChar = (char)Serial.read();
     
     // Append the char to the command received so far if there is
     // room and if there is no room, move all the characters down
     // one slot and then add it.
      
      if (commandFromComputerString.length() < commandFromComputerMaxLength)
        commandFromComputerString += currentChar;
      else
        {
          for (int i = 0; i < commandFromComputerString.length() - 1; i++)
            {
              commandFromComputerString.setCharAt(i,commandFromComputerString.charAt(i+1));
            }
          
          commandFromComputerString.setCharAt(commandFromComputerString.length() - 1,currentChar);
        }
      
      // If the character is a newline, mark the command as complete.
      
      if (currentChar == '\n')
        commandFromComputerComplete = true;
      
    }
  
}

/* Given the inject command string, the command is validated and
   the number of steps and speeds for each stage are extracted. The
   number of stages is returned and the number of steps and the
   speeds for each stage are put in injectionStageSteps and
   injectionStageSpeeds respectively. -1 is returned if the command
   is invalid.
   
   A valid command look like
   
   "Inject: A B,C D,E F"
   
   Where A, C, and E are the number of steps (+ is forward and - is
   backward) and B, D, and F are the speeds in RPM. All must be
   integers and the speeds must be positive. A space separates
   the number of steps from the speed, and a comma separates stages.
*/
int processInjectCommand(String s)
{
  
  // First check to see if the command starts right.
  
  if (!s.startsWith("Inject: "))
    return -1;
  
  // Now, remove the beginning and start processing the stages.
  
  s = s.substring(8);
  
  // Read each stage one by one, returning -1 if one is invalid or
  // the total number if the last one is read.
  
  for (int numberStages = 0; numberStages < MAX_INJECTION_STAGES; numberStages++)
    {
      
      // Each stage consists of two integers separated by a space,
      // so we need to find the space to separate the two. If
      // there is no space, then the command is invalid unless
      // there are no characters left in the string in which case
      // all that happened is that we already read the last stage.
      
      int index = s.indexOf(' ');
      
      if (index == -1)
        return s.length() == 0 ? numberStages : -1;
      
      // Get the number of steps for the stage and then remove
      // that part from the string.
      
      injectionStageSteps[numberStages] = (int)s.substring(0,index).toInt();
      
      s = s.substring(index+1);
      
      // The separator between the speed and the next stage is a
      // comma. If one is found, then there is another stage. If
      // not, this is the last stage.
      
      index = s.indexOf(',');
      
      if (index == -1)
        {
          
          // This is the last stage, so read the speed and return
          // the number of stages.
          
          injectionStageSpeeds[numberStages] = (unsigned int)s.toInt();
          return numberStages+1;
        }
      else
        {
          
          // There is another stage after this one, so read the
          // speed, remove it (and the comma) from the string to
          // go on to the next stage.
          
          injectionStageSpeeds[numberStages] = (unsigned int)s.substring(0,index).toInt();
          s = s.substring(index+1);
        }
    }
  
  // Too many stages were given if we reach here.
  
  return -1;
  
}

// Converts a float to a string.
String floatToString(float x)
{
  // We have to use the avr-libc function dtostr, which requires
  // a character array buffer. We are using the flag 0x02 which causes
  // the sign (+ or -) to always be printed.
  
  char buf[256];
  dtostre(x,buf,8,0x02);
  return String(buf);
  
}

