//
//
//
//

#include "ServoHandler.h"
#include <stdlib.h> 
#include <Arduino.h>

extern int BPM = 70; //Desired BPM
extern int quarter = 60000/BPM; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
extern int half = quarter*2; // 1/2 Note
extern int whole = quarter*4; // Whole Note
extern int eighth = quarter/2; // 1/8 Note
extern int upTime = quarter/3; // least amount of time a servo can wait before having to be ready for the next note

//**This class handles all of the information that a single servo will need to operate. Servos can operate as STRIKING SERVOS(moving from min to max range) or POSITIONAL SERVOS(moving only in the sequence of positions you give it).**// 
//**Servos can also move automatically or with manual control.**/

ServoHandler::ServoHandler() {
    range =  new int[2]{5,175}; // Servo's range
    control = 0; // 0 - Manual Control  1 - Automatic On
    mode = 0; // 0 - Striking Servo   1 - Positional Servo

    waitTime = 0; //time needed to wait before next note/position
    lastHitTime = 0; // last elapsed time of hit
    returnTime = 20;
    
    numNotes = 2; //notes in sequence
    sequence = new int[8](); //sequence of note lengths
    seqIdx = 0;//sequence position indexer
    positions = new int[2]{5,175};//list of note positions(should be congruent with sequence)
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
      break;
      
    case wholeNote:
      this->waitTime = whole;
      break;

    case eighthNote:
      this->waitTime = eighth;
      break;

    case quarterNote:
      this->waitTime = quarter;
      break;
  }
}


/***Function counts the number of notes in a sequence, then creates an array of only unique positions**/// <-- Should be run after every new parameter is input
void ServoHandler::countNotes() {
      int* sortedPos;
      sortedPos = new int [this->numNotes]();

      for(int i = 0; i<this->numNotes;i++){
        sortedPos[i] = this->positions[i];
      }     
    
      qsort(sortedPos, this->numNotes,sizeof(int),compare);

      int count = 0;
      int c, d;
      delete [] this->noDupPos;
      this->noDupPos = new int [this->numNotes](); 
      
      for(int i = 0; i<this->numNotes;i++){
       this->noDupPos[i] = sortedPos[i];
      }

      for (c = 0; c < this->numNotes; c++){
        for (d = 0; d < count; d++){
          if(sortedPos[c] == this->noDupPos[d])
            break;
              }
    if (d == count)
    {
      this->noDupPos[count] = sortedPos[c];
      count++;
    }        
}
      this->numSingleNotes = count;
}

void setBPM(int measure){ 
  
   BPM = measure; //Desired BPM
   quarter = 60000/BPM; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
   half = quarter*2; // 1/2 Note
   whole = quarter*4; // Whole Note
   eighth = quarter/2; // 1/8 Note
   upTime = quarter/3; // least amount of time a servo can wait before having to be ready for the next note

}

// qsort requires you to create a sort function
int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}
