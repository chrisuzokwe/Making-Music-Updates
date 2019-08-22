//
//
//
//

#include "ServoHandler.h"

int BPM = 90; //Desired BPM
int quarter = 60000/BPM; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
int half = quarter*2; // 1/2 Note
int whole = quarter*4; // Whole Note
int eighth = quarter/2; // 1/8 Note
int upTime = quarter/3; // least amount of time a servocan wait before having to be ready for the next note



//**This class handles all of the information that a single servo will need to operate. Servos can operate as STRIKING SERVOS(moving from min to max range) or POSITIONAL SERVOS(moving only in the sequence of positions you give it).**// 
//**Servos can also move automatically or with manual control.**/

ServoHandler::ServoHandler() {
    range =  new int[2]{80,110}; // Servo's range
    control = 0; // 0 - Manual Control  1 - Automatic On
    mode = 0; // 0 - Striking Servo   1 - Positional Servo

    waitTime = 0; //time needed to wait before next note/position
    lastHitTime = 0; // last elapsed time of hit
    
    numNotes = 0; //notes in sequence
    sequence = new int[8](); //sequence of note lengths
    seqIdx = 0;//sequence position indexer
    positions = new int[8]();//list of note positions(should be congruent with sequence)
    noDupPos = new int[8]();//list of unique notes
    numSingleNotes = 0; //unique notes in sequencs
    
  }

  
/***Function calculates time needed before next note is struck -- then steps through sequence***/
void ServoHandler::sequenceStep(){
  this->seqIdx++;
  if(this->seqIdx == this->numNotes){
    this->seqIdx = 0;
  }

  switch(this->sequence[this->seqIdx]){
    case halfNote:
      this->waitTime = half;
      this->returnTime = this->waitTime - upTime;
      break;
      
    case wholeNote:
      this->waitTime = whole;
      this->returnTime = this->waitTime - upTime;
      break;

    case eighthNote:
      this->waitTime = eighth;
      this->returnTime = this->waitTime - upTime;
      break;

    case quarterNote:
      this->waitTime = quarter;
      this->returnTime = this->waitTime - upTime;
      break;
  }
}


/***Function counts the number of notes in a sequence, then creates an array of only unique positions**/// <-- Should be run after every new parameter is input
void ServoHandler::countNotes() {
    for(int i = 0; i < 8; i++){
    if(this->sequence[i] != 0){
      this->numNotes++;}}
      this->noDupPos = this->positions;
      numSingleNotes = numNotes;

     for(int i=0; i<(numNotes-1); i++) {
        for(int o=0; o<(numNotes-(i+1)); o++) {
                if(this->noDupPos[o] > this->noDupPos[o+1]) {
                    int t = this->noDupPos[o];
                    this->noDupPos[o] = this->noDupPos[o+1];
                    this->noDupPos[o+1] = t;}}}

    for(int i=0; i<(numNotes-1);i++){

      if(this->noDupPos[i] == 0){
        break;}
      if(this->noDupPos[i] == this->noDupPos[i+1]){
        this->noDupPos[i+1] = this->noDupPos[i+2];
        numSingleNotes--;}
      else{
        continue;}}
        
}


//**set BPM and calculate new waiting times**//
void setBPM(int measure){
  
  BPM = measure; //Desired BPM
 quarter = 60000/BPM; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
 half = quarter*2; // 1/2 Note
 whole = quarter*4; // Whole Note
 eighth = quarter/2; // 1/8 Note
 upTime = quarter/3;
 
}
