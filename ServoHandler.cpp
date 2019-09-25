//A class to handle the parameters of all servos 
//
//
//

#include "ServoHandler.h"
#include <stdlib.h> 
#include <Arduino.h>

extern int BPM = 90; //Desired BPM
extern int quarter = 60000/BPM; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
extern int half = quarter*2; // 1/2 Note
extern int whole = quarter*4; // Whole Note
extern int eighth = quarter/2; // 1/8 Note
extern int upTime = quarter/3; // least amount of time a servo can wait before having to be ready for the next note

//**This class handles all of the information that a single servo will need to operate. Servos can operate as STRIKING SERVOS(moving from min to max range) or POSITIONAL SERVOS(moving only in the sequence of positions you give it).**// 
//**Servos can also move automatically or with manual control.**/

ServoHandler::ServoHandler() {
    control = 0; // 0 - Manual Control  1 - Automatic On
    mode = 0; // 0 - Standard Servo   1 - Positional Servo 2 - Stepping Servo

    waitTime = 0; //time needed to wait before next note/position
    lastHitTime = 0; // last elapsed time of hit
    returnTime = 0; // user specified swing return time (for standard+automatic servos)

    positions = new int[2]{5,175};//list of note positions(should be congruent with sequence) 
    numNotes = 2; //notes in sequence
    sequence = new int[8](); //sequence of note lengths
    seqIdx = 0;//sequence position indexer

    stepReady = 1;    
  }
  
/***Function calculates time needed before next note is struck -- then steps through sequence***/
void ServoHandler::sequenceStep(){
  this->seqIdx++;
  if(this->seqIdx == this->numNotes){
    this->seqIdx = 0;
  }
  switch(this->sequence[this->seqIdx]){
    case halfNote: //2
      this->waitTime = half;
      break;
      
    case wholeNote: //1
      this->waitTime = whole;
      break;

    case eighthNote: //8
      this->waitTime = eighth;
      break;

    case quarterNote: //4
      this->waitTime = quarter;
      break;
  }
}

/**Set the Global BPM for your sequences. The BPM then calculates the length of each individual length note.**/
void setBPM(int measure){  
   BPM = measure; //Desired BPM
   quarter = 60000/BPM; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
   half = quarter*2; // 1/2 Note
   whole = quarter*4; // Whole Note
   eighth = quarter/2; // 1/8 Note
}
