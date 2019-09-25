

/*********************************************************************
MAKING MUSIC 2019 Program Code -- Modified By Chris Uzokwe for the 2019 Apple Distinguished Educators Workshop

TODO: 
  1. Continue Testing Sequence Stepping Debounce Method
  2. Add functionality to account for rests in automatic sequence
  3. Work with swift front end to fix the "parseAndUpdate" function's intake of user data (less string packages = less chance of corruption -- so consolidate)
 

*********************************************************************/

/*=========================================================================
    APPLICATION SETTINGS & LIBRARY INCLUDES*/
  #include <Arduino.h>
  #include <SPI.h>
  #include <Servo.h>
  #include "Adafruit_BLE.h"
  #include "Adafruit_BluefruitLE_SPI.h"
  #include "Adafruit_BluefruitLE_UART.h"
  #include "BluefruitConfig.h"
  #include "ServoHandler.h"

  #if SOFTWARE_SERIAL_AVAILABLE
    #include <SoftwareSerial.h>
  #endif
    
//#define FACTORYRESET_ENABLE          NaN    // Place BLUEFRUIT in a 'known good' state for corrupted board debugging -- removed software capability, but this is still possible through hardware (DFU Pin to GND )
  #define MINIMUM_FIRMWARE_VERSION    "0.6.6" // Minimum Firmware Version need to run features
  #define MODE_LED_BEHAVIOUR          "MODE"  // LED activity, valid options are
                                              // "DISABLE" or "MODE" or "BLEUART" or
                                              // "HWUART"  or "SPI"  or "MANUAL"
  #define MSGDEBUG 0                          // View incoming string messages from Swift Playground (in function "parseAndUpdate") for communication debugging
  #define ANLOGDEBUG 0                        // Display Servo analog input and digital output for manual servo operation debugging   

  /* Initialize Bluefruit Object as hardware SPI, Bluefruit is using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
  Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
/*=========================================================================*/

/* SERVO INIT & IO */
const int pin_pairs = 4;
Servo servos[pin_pairs];
int servo_pins[pin_pairs] = {3,5,6,9}; // servo output pins
int analog_pins[pin_pairs] = {A0,A1,A2,A3}; // controller input pins
ServoHandler servo_obj[pin_pairs]; // Non-physical parameter handler for all servos
bool autoToggle = 0; // Auto Servos on(1)/off(0)

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
/**************************************************************************/
/*!@brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  
  /* SETUP SERVO PINS */
    for (int i = 0; i < pin_pairs; i++) {
    servos[i].attach(servo_pins[i]);
  }

/*==========================================================================  
 * SETUP OF THE BLE MODULE WITH DEBUG INFO -- this connection is needed before the program starts
 */
  if(Serial){ 
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  }
  
  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
//  if(Serial){
//    Serial.println( F("OK!") );
//  }  
//  /* Disable command echo from Bluefruit */
//  ble.echo(false);
//
//  if(Serial){
//    Serial.println("Requesting Bluefruit info:");
//    ble.info();  // Print Bluefruit information
//    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
//    Serial.println(F("Then Enter characters to send to Bluefruit"));
//    Serial.println();
//  }
//  ble.verbose(false);  // debug info is a little annoying after this point!
//  /* Wait for connection */
  while (! ble.isConnected()){
      delay(500);
  }
  // LED Activity command is only supported from 0.6.6 -- check before running program
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ){
    // Change Mode LED Activity
    if(Serial){
      Serial.println(F("******************************"));
      Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    }
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    if(Serial){
      Serial.println(F("******************************"));
    }
  }
/*===============================================================================================*/
}

/**************************************************************************/
// This loop handles all servo movement, while constantly polling for new parameters from the user.
/**************************************************************************/
void loop(void)
{
  for(int i = 0; i < pin_pairs; i++){ // iterate through each instantiated servo object to check its parameters and write to the servo based on its parameters
    if(servo_obj[i].control == 1){ // auto
      if(autoToggle){ // on/off switch for automated servos
        processAutomatic(servo_obj[i], i);
          }
         }
    else if(servo_obj[i].control == 0){ // manual      
        processManual(servo_obj[i], i);
          }
         }  
  digitalWrite(13,LOW); // Servo params are not being written -- indicator off

  /*Check for incoming characters from Bluefruit*/
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0){
    // no data
    return;
  }
  
  // Some data was found, its in the buffer
  #if MSGDEBUG
    Serial.print("data found...");
    Serial.print(F("[Recv] ")); 
    Serial.println(ble.buffer);
  #endif
  
  String mydata = String(ble.buffer); // load data into buffer string to parse
  parseAndUpdate(mydata);
  ble.waitForOK();
  }

/**************************************************************************/
  // Writes to specified automatic servo
/**************************************************************************/
void processAutomatic(ServoHandler &servo, int servoNum){

  unsigned long elapsed = millis(); // Check total program run time, all servos are globally dependent on this clock

  if(servo.mode == 0){ 
   if(elapsed  > servo.lastHitTime + servo.returnTime){ //
      servos[servoNum].write(servo.positions[0]); 
      }
     }

  if(elapsed - servo.lastHitTime > servo.waitTime){
    if(servo.mode == 0){
      servos[servoNum].write(servo.positions[1]);
      }
  else{
    servos[servoNum].write(servo.positions[servo.seqIdx]);  
    }
  servo.lastHitTime = elapsed;
  servo.sequenceStep();
 }  
}

/**************************************************************************/
  // Writes to specified manual servo based on its operating mode (standard, positional, stepping)
/**************************************************************************/
void processManual(ServoHandler &servo, int servoNum){

    /***MANUAL CONTROL***/
  /*each analog pin is polled so its value can be written to appropriate servo....*/

       int analog_value = analogRead(analog_pins[servoNum]); // <-- Breakout into function from here:

      #if ANLOGDEBUG
        Serial.print("SERVO: ");
        Serial.print(servoNum+1);
        Serial.print(" | MODE: ");
        Serial.print(servo.mode);
        Serial.print(" | ANALOGVAL: ");
        Serial.println(analog_value);
        Serial.print("VALUE WRITTEN: ");
      #endif        

           int servo_angle = map(
          analog_value,      // Analog value
          0,                 // Minimum analog value
          1023,              // Maximum analog value
          servo.positions[0], // Minimum servo angle
          servo.positions[1]  // Maximum servo angle
       );

       /*...STANDARD SERVOS are mapped directly to angles recieved.*/                 
          if(servo.mode == 0){
            servos[servoNum].write(servo_angle);

            #if ANLOGDEBUG
              Serial.println(servo_angle);
            #endif
          }
          
        /*.....if a servo is a QUANTIZED SERVO, depending on how many individual notes it has, potentiometer pins are locked between those values....*/
          else if(servo.mode == 1){
            int index = round( ( (float)analog_value/(float)1024.0 ) * (float)(servo.numNotes - 1) );
            servos[servoNum].write(servo.positions[index]); 
              
            #if ANLOGDEBUG
              Serial.println(servo.positions[index]);
            #endif
           }
           
         /*A Mode of 2 Means every trigger will STEP the servo through its sequence*/
         else if(servo.mode == 2){
          if(analog_value < 511){
            if(servo.stepReady){
              servos[servoNum].write(servo.positions[servo.seqIdx]);
              servo.stepReady = 0;
              servo.sequenceStep();   
             }                
            }
            else {
              servo.stepReady = 1;
            }
            #if ANLOGDEBUG
              Serial.println(servo.positions[servo.seqIdx]);
            #endif
           }  
        #if ANLOGDEBUG
         Serial.println("-------------------------------------------------------");
        #endif  // < -- to Here
  
}

/**************************************************************************/
  // decodes recieved strings and updates a servo object's parameters accordingly
/**************************************************************************/
void parseAndUpdate(String mydata){

    //get function & params
  String myfunc = mydata.substring(0,3);
  int uartServo = mydata.substring(3,4).toInt();
  mydata.remove(0,4);
  int strLength = mydata.length();

  #if MSGDEBUG
      Serial.print("SERVO: ");
      Serial.print(uartServo+1);
      Serial.print(" | FUNC: ");
      Serial.println(myfunc);
      Serial.print("REMAINING STRING: ");
      Serial.print(mydata);
      Serial.print(" | LENGTH: ");
      Serial.println(strLength);
  #endif

  
      if(myfunc == "BPM"){   
          setBPM(mydata.toInt());

      #if MSGDEBUG
          Serial.print("New BPM: ");
          Serial.println(BPM);
      #endif
      }
    
    else if(myfunc == "AUT"){   
          autoToggle = mydata.toInt();
          
          #if MSGDEBUG
            Serial.print("AUTO VAL: ");
            Serial.println(autoToggle);
          #endif
          }
    
    else if(myfunc == "POS"){
           delete [] servo_obj[uartServo].positions;
           servo_obj[uartServo].positions = new int [strLength];
           servo_obj[uartServo].numNotes = strLength;       
            
          for(int i = 0; i < strLength; i++){
            servo_obj[uartServo].positions[i] = (byte)mydata[i];
            #if MSGDEBUG
              Serial.print("Positions: ");
              Serial.println(servo_obj[uartServo].positions[i]);
            #endif
              }
            }
            
    else if(myfunc == "TIM"){
           delete [] servo_obj[uartServo].sequence;
           servo_obj[uartServo].sequence = new int [strLength];  
               
          for(int i = 0; i < strLength; i++){
            servo_obj[uartServo].sequence[i] = (byte)mydata[i];
            #if MSGDEBUG
              Serial.print("Servo timing: ");
              Serial.println(servo_obj[uartServo].sequence[i]);
            #endif
              }          
            }    
    else if(myfunc == "RET"){
            servo_obj[uartServo].returnTime = mydata.toInt();            
             #if MSGDEBUG
              Serial.print("Servo return Time: ");
              Serial.println( servo_obj[uartServo].returnTime);
             #endif
             }    
    else if(myfunc == "MOD"){
            servo_obj[uartServo].mode = mydata.toInt();
           #if MSGDEBUG
            Serial.print("Servo mode: ");
            Serial.println( servo_obj[uartServo].mode);
           #endif
           }     
    else if(myfunc == "CON"){
            servo_obj[uartServo].control = mydata.toInt();
            
            #if MSGDEBUG
              Serial.print("Servo control: ");
              Serial.println( servo_obj[uartServo].control);
            #endif
            }

                        #if MSGDEBUG
                        Serial.println("-----------------------------------------------------------------------------");                                  
                        #endif
                        }  
