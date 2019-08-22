/*********************************************************************
MAKING MUSIC 2019 Program Code -- Modified By Chris Uzokwe

TODO: 

  1. Refine Algoritithm for sorting and removing duplicates from positional array
  2. Draft function to update parameters recieved from iPad

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

/* SERVO SETTINGS */

const int pin_pairs = 4;

Servo servos[pin_pairs];

int servo_pins[pin_pairs] = {3,5,6,9}; 

int analog_pins[pin_pairs] = {A0,A1,A2,A3};

int servo_range[pin_pairs][2] = {
    { 5, 175 }, // Servo 1, pins 3-A0
    { 5, 175 }, // Servo 2, pins 5-A1
    { 5, 175 }, // Servo 3, pins 6-A2
    { 5, 175 }  // Servo 4, pins 9-A3
};

ServoHandler servo_obj[pin_pairs]; // Init parameter handler for all servos

//settings for uart incoming functions
int uartServo = 0; //servo num
int uartMax = 0; //max angle
int uartMin = 0; //min angle
int uartComma = 0;//position of comma
unsigned long elapsed = 0; //Time passed since program start
int servo_idx = 0;
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

  
/**TEST INPUT PARAMETERS
  servo_obj[servo_idx].sequence[0] = 4;
  servo_obj[servo_idx].sequence[1] = 4;
  servo_obj[servo_idx].sequence[2] = 4;
  servo_obj[servo_idx].sequence[3] = 4;
  servo_obj[servo_idx].sequence[4] = 4;
  servo_obj[servo_idx].sequence[5] = 4;
  servo_obj[servo_idx].sequence[6] = 4;

  servo_obj[servo_idx].positions[0] = 20;
  servo_obj[servo_idx].positions[1] = 30;
  servo_obj[servo_idx].positions[2] = 40;
  servo_obj[servo_idx].sequence[3] = 50;
  servo_obj[servo_idx].positions[4] = 60;
  servo_obj[servo_idx].positions[5] = 70;
  servo_obj[servo_idx].sequence[6] = 80;
  
  servo_obj[servo_idx].mode = 1;
  servo_obj[servo_idx].control = 0;

  servo_obj[1].mode = 0;
  servo_obj[1].control = 1;

  servo_obj[1].sequence[0] = 4;
  servo_obj[1].sequence[1] = 4;  
  servo_obj[1].sequence[2] = 4;
  servo_obj[1].sequence[3] = 4;
*/
  
  /* SETUP SERVO PINS */
    for (int i = 0; i < pin_pairs; i++) {
    servos[i].attach(servo_pins[i]);
    servo_obj[i].countNotes();
  }

  pinMode(13, OUTPUT); // Information recieved indicator
  
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

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

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}


/**************************************************************************/
// This loop handles all servo movement, while constantly polling for new parameters from the user.
/**************************************************************************/
void loop(void)
{
  elapsed = millis(); // Check total program run time, all servos are globally dependent on this clock 

  /***AUTOMATIC CONTROL***/
  /*for loop tells STRIKING SERVOS at what time they must rise to be ready for next note*/
  for(servo_idx = 0; servo_idx < pin_pairs; servo_idx++){
    if(servo_obj[servo_idx].control == 1){
    if(elapsed - servo_obj[servo_idx].lastHitTime > servo_obj[servo_idx].returnTime){
       if(servo_obj[servo_idx].mode == 0){
      servos[servo_idx].write(servo_obj[servo_idx].range[0]);}
    }}}
  /*for loop writes each POSITIONAL SERVO'S next desination, cycles through list*/
  for(servo_idx = 0; servo_idx < pin_pairs; servo_idx++){
    if(servo_obj[servo_idx].control == 1){
    if(elapsed - servo_obj[servo_idx].lastHitTime > servo_obj[servo_idx].waitTime){
      if(servo_obj[servo_idx].mode == 0){
      servos[servo_idx].write(servo_obj[servo_idx].range[1]);}
      else if(servo_obj[servo_idx].mode == 1){
      servos[servo_idx].write(servo_obj[servo_idx].positions[servo_obj[servo_idx].seqIdx]);  
      }
      servo_obj[servo_idx].lastHitTime = elapsed;
      servo_obj[servo_idx].sequenceStep();}}}

  
  /***MANUAL CONTROL***/
  /*each analog pin is polled so its value can be written to appropriate servo....*/
  for (int i = 0; i < pin_pairs; i++) {
      int analog_value = analogRead(analog_pins[i]);   
      int servo_angle = map(
         analog_value,      // Analog value
          0,                 // Minimum analog value
          1023,              // Maximum analog value
          servo_range[i][0], // Minimum servo angle
          servo_range[i][1]  // Maximum servo angle
       );
    if(servo_obj[i].control == 0){ 
        /*.....if a servo is a POSITIONAL SERVO, depending on how many individual notes it has, potentiometer pins are locked between those values....*/
         if(servo_obj[i].mode == 1){
          
          switch(servo_obj[i].numSingleNotes){
            
            case 1:
              servos[i].write(servo_obj[i].noDupPos[0]);
              break;

            case 2:
                if(servo_angle<90){
                servos[i].write(servo_obj[i].noDupPos[0]);}
                else if (servo_angle>90){
                servos[i].write(servo_obj[i].noDupPos[1]);}
                break;

            case 3:

                if(servo_angle>5 && servo_angle<61){
                  servos[i].write(servo_obj[i].noDupPos[0]);}
                else if(servo_angle>60 && servo_angle<116){
                  servos[i].write(servo_obj[i].noDupPos[1]);}
                else if(servo_angle>115 && servo_angle<175){
                  servos[i].write(servo_obj[i].noDupPos[2]);}
                break;

            case 4:

              if(servo_angle>5 && servo_angle<43){
                  servos[i].write(servo_obj[i].noDupPos[0]);}
                else if(servo_angle>42 && servo_angle<86){
                  servos[i].write(servo_obj[i].noDupPos[1]);}
                else if(servo_angle>85 && servo_angle<129){
                  servos[i].write(servo_obj[i].noDupPos[2]);}
                else if(servo_angle>128 && servo_angle<175){
                  servos[i].write(servo_obj[i].noDupPos[3]);}
                break;

            case 5:

            if(servo_angle>5 && servo_angle<39){
                  servos[i].write(servo_obj[i].noDupPos[0]);}
                else if(servo_angle>38 && servo_angle<73){
                  servos[i].write(servo_obj[i].noDupPos[1]);}
                else if(servo_angle>72 && servo_angle<106){
                  servos[i].write(servo_obj[i].noDupPos[2]);}
                else if(servo_angle>105 && servo_angle<140){
                  servos[i].write(servo_obj[i].noDupPos[3]);}
                else if(servo_angle>139 && servo_angle<175){
                  servos[i].write(servo_obj[i].noDupPos[4]);}
                break;

            case 6:

             if(servo_angle>5 && servo_angle<33){
                  servos[i].write(servo_obj[i].noDupPos[0]);}
                else if(servo_angle>32 && servo_angle<62){
                  servos[i].write(servo_obj[i].noDupPos[1]);}
                else if(servo_angle>61 && servo_angle<89){
                  servos[i].write(servo_obj[i].noDupPos[2]);}
                else if(servo_angle>88 && servo_angle<116){
                  servos[i].write(servo_obj[i].noDupPos[3]);}
                else if(servo_angle>115 && servo_angle<143){
                  servos[i].write(servo_obj[i].noDupPos[4]);}
                else if(servo_angle>142 && servo_angle<175){
                  servos[i].write(servo_obj[i].noDupPos[5]);}
                break;

            case 7:

               if(servo_angle>5 && servo_angle<30){
                  servos[i].write(servo_obj[i].noDupPos[0]);}
                else if(servo_angle>29 && servo_angle<55){
                  servos[i].write(servo_obj[i].noDupPos[1]);}
                else if(servo_angle>54 && servo_angle<80){
                  servos[i].write(servo_obj[i].noDupPos[2]);}
                else if(servo_angle>79 && servo_angle<105){
                  servos[i].write(servo_obj[i].noDupPos[3]);}
                else if(servo_angle>104 && servo_angle<130){
                  servos[i].write(servo_obj[i].noDupPos[4]);}
                else if(servo_angle>129 && servo_angle<155){
                  servos[i].write(servo_obj[i].noDupPos[5]);}
                else if(servo_angle>154 && servo_angle<175){
                  servos[i].write(servo_obj[i].noDupPos[6]);}
                break;

            case 8:

            if(servo_angle>5 && servo_angle<27){
                  servos[i].write(servo_obj[i].noDupPos[0]);}
                else if(servo_angle>26 && servo_angle<49){
                  servos[i].write(servo_obj[i].noDupPos[1]);}
                else if(servo_angle>48 && servo_angle<71){
                  servos[i].write(servo_obj[i].noDupPos[2]);}
                else if(servo_angle>70 && servo_angle<93){
                  servos[i].write(servo_obj[i].noDupPos[3]);}
                else if(servo_angle>92 && servo_angle<114){
                  servos[i].write(servo_obj[i].noDupPos[4]);}
                else if(servo_angle>113 && servo_angle<135){
                  servos[i].write(servo_obj[i].noDupPos[5]);}
                else if(servo_angle>134 && servo_angle<156){
                  servos[i].write(servo_obj[i].noDupPos[6]);}
                else if(servo_angle>155 && servo_angle<175){
                  servos[i].write(servo_obj[i].noDupPos[7]);}
                break;
                        }}
       /*...STRIKING SERVOS are mapped directly to angles recieved.*/                 
          else{
       servos[i].write(servo_angle);
         }}}  
   
  digitalWrite(13,LOW); // Servo params are not being written -- indicator off
  
  /*Check for user input*/
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

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
  Serial.print("data found...");
  Serial.print(F("[Recv] ")); 
  Serial.println(ble.buffer);
  String mydata = String(ble.buffer);
  //get function
  String myfunc = mydata.substring(0,2);
  //change angle function
  if (myfunc == 'ANG') { changeServoAngle(mydata); }
  ble.waitForOK();}


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
  //  Changes User Input String Into Servo Max and Min
/**************************************************************************/
bool changeServoAngle(String newAngle)
{
  //example incoming command:  ANG122,233
    //settings
    uartServo = newAngle.substring(3,4).toInt(); //servo num
    uartMax = 0; //max angle
    uartMin = 0; //min angle
    uartComma = newAngle.indexOf(',');
    int x = newAngle.length(); //length of incoming string

    //get min angle
    if(uartComma == 6) { 
        ;
        uartMin = newAngle.substring(4,6).toInt();
        uartMax = newAngle.substring(7,x).toInt();
    }
    else {
      //uartComma = "7";
      uartMin = newAngle.substring(4,7).toInt();
      uartMax = newAngle.substring(8,x).toInt();
     }
    //change servo angle
      //change servo angle array
      servo_range[uartServo][0] = uartMin;
      servo_range[uartServo][1] = uartMax; 

      //** Change Servo Params... EXAMPLE**//

     // servo_obj[uartServo].control = ctrlVal;
     // servo_obj[uartServo].mode = mdVal;

      digitalWrite(13,HIGH); // LED PIN 13 Indicates Information is Being Written...
//     );
       
}
