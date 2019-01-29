/*
    Arduino Teensy LC code for the The MD25MU0101 is a 25 Pedal, MIDI/USB, Pedalboard.
    Copyright (C) 2019  Ben Lester 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
// Do not "Autoformat" this file using the Arduino IDE.
#include <EEPROM.h>
#include <MIDI.h> // For Serial (DIN) output. USB Serial MIDI is built in
const int iEPROM_Version = 0x1001;  // Bump this to reinit with any changes. Stored at addr 0.
const int iEPROM_AddrChannel = 1;
const int iEPROM_AddrVelocity = 2;
const int iEPROM_AddrCable = 3;
const int iEPROM_AddrSensitivity = 4;
const int iEPROM_AddrTranspose = 5;
const int iMaxThreshold = 585; // This number minus Sensitivity X 10 makes a safe Threshold range.
                               // Most sensors top out at approx 618.
const int iVelocityThreshold = 455; // A safe medium when sending velocity is turned on. Sensor
                                    // reading differentials are used to calculate velocity but
                                    // due to noise this should only be done around the middle
                                    // of travel of the pedal.
const int iMinThreshold = 335;  // Minimum threashold used for maximum sensitivity.
// Note: Velocity Factor is sensitive to code changes.
// This factor * reading differintial is MIDI velocity. The sensors are scanned as fast as the CPU can
// manage to execute the code. Therefore, it is safe to assume this factor will change with any change
// to the code.
const float fVelocityFactor = 3.75;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);  // Serial1 == Teensy LC Pin 1
// Channel, Velocity flag, Cable, Sensitivity and Transpose are settings in EPROM and can be changed by
// Psalte SysEx MIDI messages.
const int iBasicChannel = 1;
byte iChannel = iBasicChannel;  // Default is 1. Range is 0 to 16
bool bSendVelocity = false;     // Calculate velocity (Curr - Prev normalized to MIDI std range)
const int iMaxVelocity = 127;   // MIDI standard maximum velocity
const int iStdVelocity = 100;   // A moderate velocity to send if Send Velocity is turned off
const int iDefaultCable = 0;    // USB MIDI specification
byte iCable = iDefaultCable;    // The USB MIDI virtual cable to use
const int iMinSensitivity = 0;  // Range is 0 to 25 (see below). Default = 0
const int iMaxSensitivity = 25;
byte iSensitivity = iMinSensitivity;  // Limit of pedal travel required to send MIDI NoteOn
const int iStdTranspose = 36;         // The leftmost pedal is C2 with no transposition.
byte iTranspose = 36;                 // Half step from 0 to 102 for the leftmost pedal. Default is 36 (C2)
const int iMaxTranspose = 102;        // This is the MIDI Limit - 25 = 102 So that the unsigned setting can
// span over the MIDI note range.
// MIDI standard sysex message identification request
// F0 7E 7F 06 01 F7
uint8_t bufIdentRequest[] = {
  0xF0, 0x7E, 0x7F,
  0x06,   // MIDI standard Sub-ID -- General Information
  0x01,   // Sub-ID2 -- Identity Request
  0xF7    // End of SysEx
};  // End of SysEx
// Psalte MIDI sysex message with which to respond to identification requests
// F0 7E 7F 06 02 1E 10 01 20 01 00 01 00 01 F7
uint8_t bufIdentReply[] = {
  0xF0, // SysEx
  0x7E, // Non-Realtime
  0x7F, // The SysEx channel = "disregard channel"
  0x06, // Sub-ID -- General Information
  0x02, // Sub-ID2 -- Identity Reply
  0x1E, // Psalte Organ Works Co. Mfg. ID
  0x10, // Pedal Board family code MSB
  0x01, // Pedal Board family code LSB
  0x20, // Pedal Board model number MSB
  0x01, // Pedal Board model number LSB
  0x00, // Software Major Version MSB
  0x01, // Software Major Version LSB
  0x00, // Software Minor Version MSB
  0x01, // Software Minor Version LSB
  0xF7  // End of SysEx
};
//   Psalte specific SysEx configuration requests received and recognized in this format:
//   Psalte Command: F0 7E <Psalte Id = 1E> <Command> <Parameter> F7
const int iDevIdNdx = 2;
const int iCmdNdx = 3;
const int iParamNdx = 4;
const unsigned int iPsalteDeviceId = 0x1E;  // Reserved MIDI Id
const byte iPsalteSetChannel = 0x01;
const byte iPsalteSetVelocity = 0x02;
const byte iPsalteSetCable = 0x03;
const byte iPsalteSetSensitivity = 0x04;
const byte iPsalteSetTranspose = 0x05;
const byte iPsalteReset = 0x10;
const byte iPsalteDumpStatus = 0x11;
// Current Settings are transmitted via this status reply message:
// F0 7E 7F 1E 01 00 00 00 24 F7
uint8_t bufStatusReply[] = {
  0xF0, // SysEx
  0x7E, // Non-Realtime
  0x7F, // The SysEx channel = "disregard channel"
  0x1E, // Psalte Organ Works Co. Mfg. ID
  0x01, // Current Channel
  0x00, // Current Velocity flag
  0x00, // Current Cable
  0x00, // Current Sensitivity
  0x24, // Current Transpose
  0xF7  // End of SysEx
};
/*
   Handler for MIDI System Exclusive messages
   Recognize two kinds of SysEx messages here
   1) The "universal" SysEx Identity Request message
   2) The Psalte specific SysEx Commands
   Note: Both are the same fixed length
   Ident Request:
      F0 7E 7F 06 01 F7
   Psalte Command: F0 7E <Psalte Id = 1E> <Command> <Parameter> F7
      F0 7E 1E 01 01 F7   Set Channel. Default =  1
      F0 7E 1E 02 00 F7   Set Velocity flag. False = Constant, True = Dynamic
      F0 7E 1E 03 00 F7   Set Virtual USB Cable. Default to 0
      F0 7E 1E 04 00 F7   Set Sensitivity.
        Allowed range 0 to 25. Default is 0. This number is multiplied by 10 then subtracted from
        600 which is the lowest sample of all sensors at fully depressed pedal to arrive at a
        threashold. Since a fully released pedal's sensor reads at most 320, subtracting the max
        sensitivity of 250 from the depressed pedal sample of 600 gives 350 which is a safe minimum
        threashold at which to send a Note On message. This sets a minimum pedal travel required to
        play the note, i.e. the position at which the pedal sounds.
      F0 7E 1E 05 00 F7   Set Transpose.
        Default value is 36. Range is 0 (transpose down 36 half steps) to MIDI Limit - 25 = 102.
        The number represents the MIDI note number to be used for the first (leftmost) pedal.
      F0 7E 1E 10 00 F7   Reset all to default values
      F0 7E 1E 11 00 F7   Dump Status (sends response Psalte SyEx msg)
*/
void onSystemExclusive(byte *msgData, unsigned int length) {
  //  Serial.println("SysEx Received");
  // Because both messages we are looking for are the same length we
  // check for that standard size.
  unsigned int iStdMsgSize = sizeof(bufIdentRequest) / sizeof(uint8_t);
  if (length != iStdMsgSize) {
    return; // only repond to standard size requests
  }
  // Check for F0 7E (System Exclusive Non-realtime) MIDI standard for first two bytes
  if ((msgData[0] != bufIdentRequest[0]) || (msgData[1] != bufIdentRequest[1])) {
    return;
  }
  // Could be a Universal Ident Request or Psalte message. Check for Ident.
  bool bIsIdentReq = true;
  unsigned int i;
  for ( i = 2; i < iStdMsgSize; i++) {
    if (msgData[i] != bufIdentRequest[i]) {
      bIsIdentReq = false;
    }
  }
  if (bIsIdentReq) {
    // repond to MIDI standard ident requests
    usbMIDI.sendSysEx(sizeof(bufIdentReply) / sizeof(uint8_t), bufIdentReply, true);
    return;
  }
  // Check for Psalte specific defined message
  if (msgData[iDevIdNdx] != iPsalteDeviceId) {
    return; // Not a Psalte SysEx message. Do nothing.
  }
  // We must have a Psalte command. Act on it or discard.
  switch (msgData[iCmdNdx]) {
    case iPsalteSetChannel:
      //      Serial.println("PSALTE_SET_CHANNEL Received");
      //      Serial.print("    Channel #");
      //      Serial.println(msgData[iParamNdx], DEC);
      if ((msgData[iParamNdx] >= 1) && (msgData[iParamNdx] <= 16)) {
        iChannel = msgData[iParamNdx];
        EEPROM.write(iEPROM_AddrChannel, iChannel);
      }
      break;
    case iPsalteSetVelocity:
      //      Serial.println("PSALTE_SET_VELOCITY Received");
      //      Serial.print("    On/Off: ");
      //      Serial.println(msgData[iParamNdx], DEC);
      if (msgData[iParamNdx] > 0) {
        bSendVelocity = true;
      } else {
        bSendVelocity = false;
      }
      EEPROM.write(iEPROM_AddrVelocity, bSendVelocity);
      break;
    case iPsalteSetCable:
      //      Serial.println("PSALTE_SET_CABLE Received");
      //      Serial.print("    Cable #");
      //      Serial.println(msgData[iParamNdx], DEC);
      if ((msgData[iParamNdx] >= 0) && (msgData[iParamNdx] <= 16)) {
        iCable = msgData[iParamNdx];
        EEPROM.write(iEPROM_AddrCable, iCable);
      }
      break;
    case iPsalteSetSensitivity:
      //      Serial.println("PSALTE_SET_SENSITIVITY Received");
      //      Serial.print("    Value: ");
      //      Serial.println(msgData[iParamNdx], DEC);
      if (msgData[iParamNdx] <= iMaxSensitivity) {  // Range is 0 to 25
        iSensitivity = msgData[iParamNdx];
        EEPROM.write(iEPROM_AddrSensitivity, iSensitivity);
      }
      break;
    case iPsalteSetTranspose:
      //      Serial.println("PSALTE_SET_TRANSPOSE Received");
      //      Serial.print("    Value: ");
      //      Serial.println(msgData[iParamNdx], DEC);
      if (msgData[iParamNdx] <= iMaxTranspose) {  // Range is 0 to 102
        iTranspose = msgData[iParamNdx];
        EEPROM.write(iEPROM_AddrTranspose, iTranspose);
      }
      break;
    case iPsalteReset:
      // Serial.println("PSALTE_RESET Received");
      // Set initial EPROM settings
      setInitialEPROM_Settings();
      break;
    case iPsalteDumpStatus:
      {
        //Serial.println("PSALTE_DUMP_STATUS Received");
        // Psalte Dump Status sysex message
        bufStatusReply[4] = iChannel;
        bufStatusReply[5] = bSendVelocity;
        bufStatusReply[6] = iCable;
        bufStatusReply[7] = iSensitivity;
        bufStatusReply[8] = iTranspose;
        usbMIDI.sendSysEx(sizeof(bufStatusReply) / sizeof(uint8_t), bufStatusReply, true);
      }
      break;
      //    default : // Unrecognized command was sent to our device Id. Discard (ignore).
      //      Serial.println("Unrecognized command was sent to our device Id. Discarded");
  }
}
void setInitialEPROM_Settings() {
  // Set to all defaults
  iChannel = iBasicChannel;
  bSendVelocity = false;
  iCable = iDefaultCable;
  iSensitivity = iMinSensitivity;
  iTranspose = iStdTranspose;
  // Write defaults to EPROM
  EEPROM.write(iEPROM_AddrChannel, iChannel);
  EEPROM.write(iEPROM_AddrVelocity, bSendVelocity);
  EEPROM.write(iEPROM_AddrCable, iCable);
  EEPROM.write(iEPROM_AddrSensitivity, iSensitivity);
  EEPROM.write(iEPROM_AddrTranspose, iTranspose);
}
//   Multiplexed (Muxed) Analog Inputs come from 3 multiplexers. "Middle" C3, is wired direct to an input pin
const int iSelectOnePin = 12;
const int iSelectTwoPin = 11;
const int iSelectThreePin = 10;
const int iMiddleCPin = 14;
const int iAnalogOnePin = 15;
const int iAnalogTwoPin = 16;
const int iAnalogThreePin = 17;
const int iSelectPinCount = 3;
const byte inputPinCount = 3;
const int zSelectPins[iSelectPinCount] = {iSelectOnePin, iSelectTwoPin, iSelectThreePin}; // S0~12, S1~11, S2~10
const int zInputPins[inputPinCount] = {iAnalogOnePin, iAnalogTwoPin, iAnalogThreePin};
const byte muxSelectCount = 8;  // Count of multiplexed inputs per multiplexer
const int iMuxedInputsCount = 24; // 3 Multiplexers X 8 ea. inputs
typedef struct {
  int iPrevSense;   // Previously recorded sensor value
  int iCurrSense;   // Current recorded sensor value
  bool bOn;         // Has there been a Note On message sent without a corresponding Note Off?
} SENSE_INPUT_STATUS_t;
// Note: We add an array element to the end here because Middle C is not multiplexed. This is
// because there are 25 pedals and each multiplexor can multiplex 8 inputs so only three
// multiplexors are necessary (3 X 8 = 24) leaving only one note to wire directly to an input.
// That note is C3 called here the "middle" C only because it is the C note between the other
// two C's that are the first and last pedals.
SENSE_INPUT_STATUS_t zInputStatus[iMuxedInputsCount + 1];
// "Middle" C is the last input array element
const int iMiddleCNdx = sizeof(zInputStatus) / sizeof(SENSE_INPUT_STATUS_t) - 1;
// Sensor Setup
void sensorSetup() {
  for (byte i = 0; i < iSelectPinCount; i++)
  {
    pinMode(zSelectPins[i], OUTPUT);
    digitalWrite(zSelectPins[i], HIGH);
  }
  for (byte inputPin = 0; inputPin < inputPinCount; inputPin++) {
    pinMode(zInputPins[inputPin], INPUT);
  }
}
// Read config settings from EEPROM. If address 0 doesn't have the magic version set
// all settings to defaults. If the magic number is there, retrieve the settings.
void EPROM_Setup() {
  int iVer = EEPROM.read(0);
  if (iVer != iEPROM_Version) {
    setInitialEPROM_Settings();
  } else {
    iChannel = EEPROM.read(iEPROM_AddrChannel);
    bSendVelocity = EEPROM.read(iEPROM_AddrVelocity);
    iCable = EEPROM.read(iEPROM_AddrCable);
    iSensitivity = EEPROM.read(iEPROM_AddrSensitivity);
    iTranspose = EEPROM.read(iEPROM_AddrTranspose);
  }
}
/*
   Teensy Setup
        Read EPROM values
        Set pin modes
        Register MIDI callback functions
        Read initial sensor values (both muxed and middle C)
*/
void setup() {
  EPROM_Setup();
  MIDI.begin(); // No need to use Serial.begin(31250)
  sensorSetup();
  usbMIDI.setHandleSystemExclusive(onSystemExclusive);
  updateMuxedReadings();  // Initial read of sensors primes the pump.
  updateMiddleCReadings();
}
// The selectMuxPin function sets the S0, S1, and S2 pins
// accordingly, given a pin from 0-7.
void selectMuxPin(byte pin)
{
  for (int i = 0; i < iSelectPinCount; i++)
  {
    if (pin & (1 << i))
      digitalWrite(zSelectPins[i], HIGH);
    else
      digitalWrite(zSelectPins[i], LOW);
  }
}
// Read multiplexed sensors retaining previous reading
void updateMuxedReadings() {
  int iInputStatusNdx = 0;
  for (byte muxPin = 0; muxPin <= muxSelectCount - 1; muxPin++)
  {
    selectMuxPin(muxPin); // Select one at a time
    //    Serial.print(String(muxPin) + "\t");
    for (byte inputPin = 0; inputPin < inputPinCount; inputPin++) {
      int inputValue = analogRead(zInputPins[inputPin]); // and read Z
      //      Serial.print(String(inputValue) + "\t");
      zInputStatus[iInputStatusNdx].iPrevSense = zInputStatus[iInputStatusNdx].iCurrSense;
      zInputStatus[iInputStatusNdx].iCurrSense = inputValue;
      if (iInputStatusNdx < iMuxedInputsCount) {
        iInputStatusNdx++;
      } else {
        iInputStatusNdx = 0;
      }
    }
    //    Serial.println("------------------------------");
  }
}
// Read "middle" C3 directly from an input pin.
void updateMiddleCReadings() {
  int inputValue = analogRead(iMiddleCPin);
  //      Serial.print(String(inputValue) + "\t");
  zInputStatus[iMiddleCNdx].iPrevSense = zInputStatus[iMiddleCNdx].iCurrSense;
  zInputStatus[iMiddleCNdx].iCurrSense = inputValue;
}
int zNoteNumbers[] = {
/*
  MIDI
  Note
  No.   Ndx   Name */
  36, // 00   C2
  44, // 01   G#2
  53, // 02   F3

  37, // 03   C#2
  45, // 04   A2
  54, // 05   F#3

  38, // 06   D2
  46, // 07   A#2
  55, // 08   G3

  39, // 09   D#2
  47, // 10   B2
  56, // 11   G#3

  40, // 12   E2
  49, // 13   C#3 We have skiped C3 here. It's not muxed but wired direct instead.
  57, // 14   A3

  41, // 15   F2
  50, // 16   D3
  58, // 17   A#3

  42, // 18   F#2
  51, // 19   D#3
  59, // 20   B3

  43, // 21   G2
  52, // 22   E3
  60, // 23   C4

  48  // 24   C3
};
// Send any/all MIDI Notes on/off messages when both previous and current readings
// indicate above/below threashold ammounts.
void updateMIDI(int iThreshold) {
  for (int i = 0; i < iMuxedInputsCount; i++) {
    sendMIDINote(i, iThreshold);
  }
  sendMIDINote(iMiddleCNdx, iThreshold);
}
//  Debug only, referenced by velocity dump below:
//  int iNoteCount = 0;
//
//  Check One Note and send MIDI message if appropriate
void sendMIDINote(int iNdx, int iThreshold) {
  int iVel = iStdVelocity;
  if (zInputStatus[iNdx].iCurrSense > iThreshold) {
    if (zInputStatus[iNdx].iPrevSense > iThreshold) {
      if (!zInputStatus[iNdx].bOn) {
        if(bSendVelocity) {
          int iDiff = max(zInputStatus[iNdx].iCurrSense - zInputStatus[iNdx].iPrevSense, 0);
          iVel = max(min(iDiff * fVelocityFactor, iMaxVelocity), 1);
          //Serial.printf("[%d] Velocity: %d\r\n", iNoteCount++, iVel);
        } else {
          iVel = iStdVelocity;
        }
        usbMIDI.sendNoteOn(transposeNote(zNoteNumbers[iNdx]), iVel, iChannel, iCable);
        MIDI.sendNoteOn(transposeNote(zNoteNumbers[iNdx]), iVel, iChannel);
        zInputStatus[iNdx].bOn = true;
      }
    }
  }
  if (zInputStatus[iNdx].iCurrSense < iThreshold) {
    if (zInputStatus[iNdx].iPrevSense < iThreshold) {
      if (zInputStatus[iNdx].bOn) {
        if(bSendVelocity) {
          int iDiff = max(zInputStatus[iNdx].iPrevSense - zInputStatus[iNdx].iCurrSense, 0);
          iVel = max(min(iDiff * fVelocityFactor, iMaxVelocity), 1);
          //Serial.printf("[%d] Velocity: %d\r\n", iNoteCount++, iVel);
        } else {
          iVel = iStdVelocity;
        }
        usbMIDI.sendNoteOff(transposeNote(zNoteNumbers[iNdx]), iVel, iChannel, iCable);
        MIDI.sendNoteOff(transposeNote(zNoteNumbers[iNdx]), iVel, iChannel);
        zInputStatus[iNdx].bOn = false;
      }
    }
  }
}
int transposeNote(int stdTuningNote) {
  return stdTuningNote + (iTranspose - iStdTranspose);
}
/*
   Main Teensy execution Loop
   The timming of this loop is important. The Velocity (if sent) is calculated based on the time
   it takes for this loop to execute. Any change to this timing effects the "const float fVelocityFactor = 3.75;"
   declaration above. Any change should be followed by examining the effect on MIDI velocity numbers in actual
   practice and adjusting that value appropriately.
*/
void loop() {
  usbMIDI.read();   // Check for System Exclusive requests such as Ident and Psalte commands
  updateMuxedReadings();    // Store sensor readings from multiplexors
  updateMiddleCReadings();  // Handle special case of non-multiplexed "middle" C3
  // Send MIDI note on/off messages. There is more noise at the higher and lower readings.
  // This noise interfears with velocity calculations but goes unnoticed in threashold calculations
  // so we fix the threashold at a safe middle value only when velocity is turned on.
  if (bSendVelocity) {
    updateMIDI(iVelocityThreshold);
  } else {
    updateMIDI(iMaxThreshold - (iSensitivity * 10));
  }
  //displayReadings();
}
// Debug purposes only
void displayReadings() {
  delay(500); // Must delay to help serial monitor cactch up.
  Serial.print("\r\n\r\n            ");
  for (int sp = 0; sp < 65; sp++) {
    if (sp % 10 == 0) {
      Serial.printf("%d", (int)round(ceil(sp / 10)));
    } else {
      Serial.print(".");
    }
  }
  Serial.println();
  int iSz = sizeof(zInputStatus) / sizeof(SENSE_INPUT_STATUS_t);
  for (int i = 0; i < iSz; i++) {
    Serial.printf("[%2d] (%3d) [", i, zInputStatus[i].iCurrSense);
    for (int j = 0; j < zInputStatus[i].iCurrSense / 10; j++) {
      Serial.print(">");
    }
    Serial.println("X");
  }
}


