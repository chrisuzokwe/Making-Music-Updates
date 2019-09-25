 

//
// Handler for Servo Sequences - Making Music
//
// Created by Chris Uzokwe on 8/15/2019

#ifndef ServoHandler_h
#define ServoHandler_h

#include <stdio.h>

extern int BPM; //Desired BPM
extern int quarter; //BPM to Millis (Beat Length) (4/4 Time) (1/4th Note)
extern int half; // 1/2 Note
extern int whole; // Whole Note
extern int eighth; // 1/8 Note
extern int upTime; // least amount of time a servo can wait before having to be ready for the next note


enum noteLength {
  halfNote = 2,
  wholeNote = 1,
  quarterNote = 4,
  eighthNote = 8
};

class ServoHandler {

  public:
    ServoHandler();
        
    bool control; // 0 - Manual Control   1 - Sequencer On
    int mode; // 0 - Striking Servo   1 - Positional Servo 2 - Stepping Servo

    int waitTime;
    unsigned long lastHitTime;
    unsigned long returnTime;
    
    int numNotes;
    int *sequence; //
    int *positions; //
    int seqIdx;
    void sequenceStep(); //
    void countNotes(); //

    bool stepReady;    
        
};

void setBPM(int measure);
    
    
#endif /*ServoHandler_h*/
