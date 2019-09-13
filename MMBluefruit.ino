

/*********************************************************************
MAKING MUSIC 2019 Program Code -- Modified By Chris Uzokwe

TODO: 

  1. Refine Algoritithm for sorting and removing duplicate servo positions from positional array (divide by number)
  2. Add functionality to account for rests
  3. rm range
  4. manual stuff first - map to range & map to positions and step
  

Relevant Licenses and Contributions :
  
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

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

#define MSGDEBUG 0
#define ANLOGDEBUG 0

/* SERVO INIT & IO */

const int pin_pairs = 4;
Servo servos[pin_pairs];
int servo_pins[pin_pairs] = {3,5,6,9}; 
int analog_pins[pin_pairs] = {A0,A1,A2,A3};
ServoHandler servo_obj[pin_pairs]; // Non-physical parameter handler for all servos

unsigned long elapsed = 0; //Time passed since program start
int servo_idx = 0; //Indexing variable
bool autoToggle = 0; // Auto Servos on(1)/off(0)
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
//Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

  Serial.begin(115200);
  
  /* SETUP SERVO PINS */
    for (int i = 0; i < pin_pairs; i++) {
    servos[i].attach(servo_pins[i]);
    //servo_obj[i].countNotes();
  }

  pinMode(13, OUTPUT); // Information Writing indicator

  if(Serial){
  while (!Serial);  // required for Flora & Micro
  delay(500); 
  
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  }
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if(Serial){
  Serial.println( F("OK!") );
  }

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

if(Serial){
  Serial.println("Requesting Bluefruit info:");

  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
}

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }


  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
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
}


/**************************************************************************/
// This loop handles all servo movement, while constantly polling for new parameters from the user.
/**************************************************************************/
void loop(void)
{
  elapsed = millis(); // Check total program run time, all servos are globally dependent on this clock 
  
  /***AUTOMATIC CONTROL***/
  if(autoToggle){ /*turn on or off automatic servos
  /*for loop tells STRIKING SERVOS at what time they must rise to be ready for next note*/
  for(servo_idx = 0; servo_idx < pin_pairs; servo_idx++){
    if(servo_obj[servo_idx].control == 1){
      if(elapsed  > servo_obj[servo_idx].lastHitTime + servo_obj[servo_idx].returnTime){
         if(servo_obj[servo_idx].mode == 0){
            servos[servo_idx].write(servo_obj[servo_idx].positions[0]);
            }
           }
          }
         }
    
  /*for loop tells STRIKING SERVO to hit, or writes each POSITIONAL SERVO'S next destination and cycles through list*/
  for(servo_idx = 0; servo_idx < pin_pairs; servo_idx++){
    if(servo_obj[servo_idx].control == 1){
      if(elapsed - servo_obj[servo_idx].lastHitTime > servo_obj[servo_idx].waitTime){
        if(servo_obj[servo_idx].mode == 0){
          servos[servo_idx].write(servo_obj[servo_idx].positions[1]);
          }
        else if(servo_obj[servo_idx].mode == 1){
          servos[servo_idx].write(servo_obj[servo_idx].positions[servo_obj[servo_idx].seqIdx]);  
          }
        servo_obj[servo_idx].lastHitTime = elapsed;
        servo_obj[servo_idx].sequenceStep();
       }
      }
     }
    }
  
  /***MANUAL CONTROL***/
  /*each analog pin is polled so its value can be written to appropriate servo....*/
  for (int i = 0; i < pin_pairs; i++) {

      int analog_value = analogRead(analog_pins[i]);

      #if ANLOGDEBUG
        Serial.print("SERVO: ");
        Serial.print(i+1);
        Serial.print(" | MODE: ");
        Serial.print(servo_obj[i].mode);
        Serial.print(" | ANALOGVAL: ");
        Serial.println(analog_value);
        Serial.print("VALUE WRITTEN: ");
      #endif        

           int servo_angle = map(
          analog_value,      // Analog value
          0,                 // Minimum analog value
          1023,              // Maximum analog value
          servo_obj[i].positions[0], // Minimum servo angle
          servo_obj[i].positions[1]  // Maximum servo angle
       );

    if(servo_obj[i].control == 0){ 
       /*...STANDARD SERVOS are mapped directly to angles recieved.*/                 
          if(servo_obj[i].mode == 0){
            servos[i].write(servo_angle);

            #if ANLOGDEBUG
              Serial.println(servo_angle);
            #endif
          }
          
        /*.....if a servo is a QUANTIZED SERVO, depending on how many individual notes it has, potentiometer pins are locked between those values....*/
          else if(servo_obj[i].mode == 1){
            int index = round( ( (float)analog_value/(float)1024.0 ) * (float)(servo_obj[i].numNotes - 1) );
            servos[i].write(servo_obj[i].positions[index]); 
              
            #if ANLOGDEBUG
              Serial.println(servo_obj[i].positions[index]);
            #endif
           }
           
         /*A Mode of 2 Means every trigger will STEP the servo through its sequence*/
         else if(servo_obj[i].mode == 2){
          if(analog_value < 511){
            servos[i].write(servo_obj[i].positions[servo_obj[i].seqIdx]);
            
            #if ANLOGDEBUG
              Serial.println(servo_obj[i].positions[servo_obj[i].seqIdx]);
            #endif
            
            servo_obj[i].sequenceStep();       
            }
           }  
          }
          #if ANLOGDEBUG
           Serial.println("-------------------------------------------------------");
          #endif
         }
   
  digitalWrite(13,LOW); // Servo params are not being written -- indicator off
  
  /*Check for user input*/
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
#if MSGDEBUG
    Serial.print("[Send] ");
    Serial.println(inputs);
#endif

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }}

  /*Check for incoming characters from Bluefruit*/
  ble.println("AT+BLEUARTRX");
  
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  
  // Some data was found, its in the buffer
  #if MSGDEBUG
    Serial.print("data found...");
    Serial.print(F("[Recv] ")); 
    Serial.println(ble.buffer);
  #endif
  
  String mydata = String(ble.buffer);
  parseAndUpdate(mydata);
  ble.waitForOK();
  //Serial.flush();
  }


/**************************************************************************/
  //  @brief  Checks for user input (via the Serial Monitor)
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);
  memset(buffer, 0, maxSize);
  
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }
  
  if ( timeout.expired() ) return false;
  uint8_t count=0;
  
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
  } while( (count < maxSize) && (Serial.available()) );
  return true;
}



/**************************************************************************/
  // decodes recieved strings and updates parameters accordingly
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
          
    else if(myfunc == "ANG"){
        servo_obj[uartServo].range[0] = mydata.substring(0,mydata.indexOf(',')).toInt();
        servo_obj[uartServo].range[1] = mydata.substring(mydata.indexOf(','),mydata.length()).toInt();

        #if MSGDEBUG
          Serial.print("SERVO RNG: ");
          Serial.print(servo_obj[uartServo].range[0]);
          Serial.print(" to ");
          Serial.println(servo_obj[uartServo].range[1]);
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
            servo_obj[uartServo].countNotes();
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
            servo_obj[uartServo].countNotes();
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
