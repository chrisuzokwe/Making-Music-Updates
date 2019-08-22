

//
// Handler for Servo Sequences - Making Music
//
// Created by Chris Uzokwe on 8/15/2019

#ifndef ServoHandler_h
#define ServoHandler_h

#include <stdio.h>


enum noteLength {
  halfNote = 2,
  wholeNote = 1,
  quarterNote = 4,
  eighthNote = 8
};

class ServoHandler {

  public:
    ServoHandler();
    int *range;
   
    //int servoNum; Not Sure if this is needed
    
    bool control = 0; // 0 - Manual Control   1 - Sequencer On
    bool mode = 0; // 0 - Striking Servo   1 - Positional Servo

    int waitTime = 0;
    unsigned long lastHitTime = 0;
    int returnTime = 0;
    
    int numNotes = 0;
    int numSingleNotes = 0;
    int *sequence; //
    int *positions; //
    int *noDupPos; // 
    int seqIdx = 0;
    void sequenceStep(); //
    void countNotes(); //

        
        
};

    
    
#endif /*ServoHandler_h*/
