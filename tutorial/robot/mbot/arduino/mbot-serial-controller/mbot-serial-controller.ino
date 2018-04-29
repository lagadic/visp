#include <Arduino.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeSerial.h>

// Motor controller timeout
unsigned long timeout_ms = 1000; // Timeout milli sec
unsigned long time_last_cmd = 0;

// Encoder Motor
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

// Me Auriga hardware serial port to dial with Raspberry
MeSerial mySerial(PORT_5); 

// LED ring
MeRGBLed rgbled(0, 12);

// Verbose and debug flags
bool verbose = true;
bool debug = false;

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }else{
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus();
  }else{
    Encoder_2.pulsePosPlus();
  }
}

/**
 * \par Function
 *   getKey
 * \par Description
 *   Decode a string which has the following format "${key}${separator}${value}"
 * \param[in]
 *   string - The string to decode.
 * \param[in]
 *   separator - The one char separator between the key and its value.
 * \return
 *   The ${key} string.
 */
String getKey(String string, String separator)
{
  String key;
  if(string.length()>2) {
    char * tmp;
    char * str;
    str = strtok_r((char*)string.c_str(), separator.c_str(), &tmp);
    key = String(str);
  }
  return key;
}

/**
 * \par Function
 *   getKey
 * \par Description
 *   Decode a string which has the following format "${key}${separator}${value}"
 * \param[in]
 *   string - The string to decode.
 * \param[in]
 *   key - The key to decode.
 * \param[in]
 *   separator - The one char separator between the key and its value.
 * \return
 *   The ${value} string.
 */
String getValue(String string, String key, String separator)
{
  String val;
  if(string.length()>2) {
    char * tmp;
    char * str;
    str = strtok_r((char*)string.c_str(), separator.c_str(), &tmp);
    if(str!=NULL && strcmp(str,key.c_str())==0) {
      val = String(tmp);
    }
  }
  return val;
}

void setup() {
  // Serial connexion with Raspberry
  mySerial.begin(115200);   // Opens serial port at 115200 bps
  
  // Serial connexion with console used for debug info
  Serial.begin(115200);     // Opens serial port at 9600 bps
  
  // Led ring
  rgbled.setpin(44);
  rgbled.setColor(0, 0, 0, 0); // Turn all LED off
  rgbled.show();
  
  // Motors - Set Pwm 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Encoder_1.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setPulse(9);
  Encoder_2.setRatio(39.267);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_2.setSpeedPid(0.18,0,0);

  // Init time last cmd received
  time_last_cmd = millis();
}

void loop() {

  Encoder_1.loop();
  Encoder_2.loop();

  if (mySerial.dataLineAvailable() > 0) {
    String separator("=");
    String data = mySerial.readDataLine();
    String key = getKey(data, separator);
    if (verbose) {
      Serial.print("Receive: ");
      Serial.println(data);
    }
    if (debug) {
      Serial.print("key: ");
      Serial.println(key);
    }
    if (key == "MOTOR_RPM") {
      String value = getValue(data, key, separator);
      if (verbose) {
        Serial.print("decode: ");
        Serial.println(value);
      }
      String first_token = getKey(value, ",");
      String second_token = getValue(value, first_token, ",");
      if (debug) {
        Serial.print("left cmd: ");
        Serial.println(first_token);
        Serial.print("right cmd: ");
        Serial.println(second_token);
      }
      Encoder_1.runSpeed(atoi(first_token.c_str()));
      Encoder_2.runSpeed(atoi(second_token.c_str()));

      time_last_cmd = millis();
    }
    else if (key == "LED_RING"){
      String string_rgb = getValue(data, key, separator);
      if (verbose) {
        Serial.print("Decode: ");
        Serial.println(string_rgb);
      }
      int rgb[4]; // index, r, g, b
      String token[4];
      int i = 0;
      for(i=0; i<3; i++) {
        token[i] = getKey(string_rgb, ",");
        rgb[i] = atoi(token[i].c_str());
        string_rgb = getValue(string_rgb, token[i], ",");
        if (debug) {
          Serial.print("token: ");
          Serial.println(token[i]);
          Serial.print("string_rgb: ");
          Serial.println(string_rgb);
          Serial.print("led val: ");
          Serial.println(rgb[i], DEC);
        }
      }
      rgb[3] = atoi(string_rgb.c_str());
      
      rgbled.setColor(rgb[0], rgb[1], rgb[2], rgb[3]);
      rgbled.show();
    }
  }

  // Timeout check
  if (millis() - time_last_cmd > timeout_ms) {
    // Stop motors
    Encoder_1.runSpeed(0);
    Encoder_2.runSpeed(0);
    rgbled.setColor(6, 1, 0, 0);   // Left LED turn RED
    rgbled.setColor(12, 1, 0, 0);  // Right LED turn RED
    rgbled.show();
  }
  else {
    rgbled.setColor(6, 0, 1, 0);   // Left LED turn GREEN
    rgbled.setColor(12, 0, 1, 0);  // Right LED turn GREEN
    rgbled.show();    
  }
}
