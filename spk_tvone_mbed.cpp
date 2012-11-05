// *spark audio-visual
// RS232 Control for TV-One products
// Good for 1T-C2-750, others will need some extra work

/* Copyright (c) 2011 Toby Harris, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "spk_tvone_mbed.h"
#include "mbed.h"

SPKTVOne::SPKTVOne(PinName txPin, PinName rxPin, PinName signWritePin, PinName signErrorPin, Serial *debugSerial)
{
    // Create Serial connection for TVOne unit comms
    // Creating our own as this is exclusively for TVOne comms
    serial = new Serial(txPin, rxPin);
    serial->baud(57600);
    
    if (signWritePin != NC) writeDO = new DigitalOut(signWritePin);
    else writeDO = NULL;
    
    if (signErrorPin != NC) errorDO = new DigitalOut(signErrorPin);
    else errorDO = NULL;
    
    timeoutCommandPeriod = 100;
    minimumCommandPeriod = 30;
    
    timer.start();
    
    // Link up debug Serial object
    // Passing in shared object as debugging is shared between all DVI mixer functions
    debug = debugSerial;
}

bool SPKTVOne::command(uint8_t channel, uint8_t window, int32_t func, int32_t payload)
{
    int ackBuff[standardAckLength] = {0};
    
    bool success = command(writeCommandType, ackBuff, standardAckLength, channel, window, func, payload);
    
    // TASK: Check return payload is what we tried to set it to
    char payloadStr[7];
    for (int i = 0; i < 6; i++)
    {
        payloadStr[i] = ackBuff[11+i];
    }
    payloadStr[6] = NULL;
    
    int payloadBack = strtol (payloadStr, NULL, 16);
    
    if (payload != payloadBack)
    {
        success = false;
        if (debug) debug->printf("TVOne return value (%d) is not what was set (%d). Channel: %#x, Window: %#x, Function: %#x \r\n", payloadBack, payload, channel, window, func); 
    }
    return success;
}

bool SPKTVOne::readCommand(uint8_t channel, uint8_t window, int32_t func, int32_t &payload)
{
    int ackBuff[standardAckLength] = {0};
    
    bool success = command(readCommandType, ackBuff, standardAckLength, channel, window, func, payload);
    
    char payloadStr[7];
    for (int i = 0; i < 6; i++)
    {
        payloadStr[i] = ackBuff[11+i];
    }
    payloadStr[6] = NULL;
    
    payload = strtol (payloadStr, NULL, 16);
    
    return success;
}

bool SPKTVOne::command(commandType readWrite, int* ackBuffer, int ackLength, uint8_t channel, uint8_t window, int32_t func, int32_t payload) 
{ 
  if (debug) debug->printf("TVOne %s Channel: %#x, Window: %#x, Function: %#x Payload: %i \r\n", (readWrite == writeCommandType) ? "Write" : "Read", channel, window, func, payload);

  // TASK: Sign start of serial command write
  if (writeDO) *writeDO = 1;
  
  // TASK: Prepare to issue command to the TVOne unit
  // - discard anything waiting to be read in the return serial buffer
  // - make sure we're past the minimum time between command sends as the unit can get overloaded
  while (serial->readable() || timer.read_ms() < minimumCommandPeriod) {
    if (serial->readable()) serial->getc();
  }
  
  // TASK: Create the bytes of command

  uint8_t cmd[8];
  uint8_t checksum = 0;

  // CMD
  cmd[0] = readWrite<<7 | 1<<2;
  // CHA
  cmd[1] = channel;
  // WINDOW
  cmd[2] = window;
  // OUTPUT & FUNCTION
  //            cmd[3]  cmd[4]
  // output 0 = 0000xxx xxxxxxx
  // function = xxxXXXX XXXXXXX
  cmd[3] = func >> 8;
  cmd[4] = func & 0xFF;
  // PAYLOAD
  cmd[5] = (payload >> 16) & 0xFF;
  cmd[6] = (payload >> 8) & 0xFF;
  cmd[7] = payload & 0xFF;

  // TASK: Write the bytes of command to RS232 as correctly packaged 20 characters of ASCII
  
  if (readWrite == writeCommandType)
  {
    for (int i=0; i<8; i++) checksum += cmd[i];
    serial->printf("F%02X%02X%02X%02X%02X%02X%02X%02X%02X\r", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7], checksum);
  }
  if (readWrite == readCommandType)
  {
    for (int i=0; i<5; i++) checksum += cmd[i];
    serial->printf("F%02X%02X%02X%02X%02X%02X\r", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], checksum);
  } 
   
  // TASK: Check the unit's return string, to enable return to main program as soon as unit is ready

  // Handling the timing of this return is critical to effective control.
  // Returning the instant something is received back overloads the processor, as does anything until the full 20 char acknowledgement.
  // TVOne turn out to say that receipt of the ack doesn't guarantee the unit is ready for the next command. 
  // According to the manual, operations typically take 30ms, and to simplify programming you can throttle commands to every 100ms.
  // 100ms is too slow for us. Going with returning after 30ms if we've received an acknowledgement, returning after 100ms otherwise.
  
  bool success = false;  
  int ackPos = 0;
  timer.reset();

  while (timer.read_ms() < timeoutCommandPeriod) 
  {
    if (serial->readable())
    {
        if (ackPos == 0)
        {
            ackBuffer[0] = serial->getc();
            if (ackBuffer[0] == 'F') ackPos = 1;
        }
        else
        {
            ackBuffer[ackPos] = serial->getc();
            ackPos++;
            if (ackPos == ackLength) break;
        }
    }
  }
  
  // Return true if we got the no error acknowledgement from the unit. The rest of the ack will be verified elsewhere if needed.
  if (ackPos > 2 && ackBuffer[1] == '4') 
  {
     success = true;
  }
  
  // TASK: Sign end of write
  
  if (writeDO) *writeDO = 0;
  
  if (!success) {
        if (errorDO) {
            signErrorTimeout.detach();
            signErrorTimeout.attach(this, &SPKTVOne::signErrorOff, 0.25);
            *errorDO = 1;
        }
        
        if (debug) {
            debug->printf("TVOne serial error. Time from finishing writing command: %ims. Received %i ack chars:", timer.read_ms(), ackPos);
            for (int i = 0; i<ackLength; i++) 
            {
                debug->printf("%c", ackBuffer[i]);
            }
            debug->printf("\r\n");
        }
  };

  return success;
}

int  SPKTVOne::millisSinceLastCommandSent()
{
    return timer.read_ms();
}

bool SPKTVOne::setCustomResolutions() 
{
  bool ok = true;;
  int unlocked = 0;
  int locked = 1;
  
  ok = ok && command(0, 0, kTV1FunctionAdjustFrontPanelLock, locked);
  
  ok = ok && set1920x480(kTV1ResolutionTripleHeadVGAp60);
  ok = ok && set1600x600(kTV1ResolutionDualHeadSVGAp60);
  ok = ok && set2048x768(kTV1ResolutionDualHeadXGAp60);
  
  ok = ok && command(0, 0, kTV1FunctionAdjustFrontPanelLock, unlocked);
  
  return ok;
}

bool SPKTVOne::setHDCPOn(bool state) 
{
  bool ok;

  // Turn HDCP off on the output
  ok =       command(0, kTV1WindowIDA, kTV1FunctionAdjustOutputsHDCPRequired, state);
  ok = ok && command(0, kTV1WindowIDA, kTV1FunctionAdjustOutputsHDCPStatus, state);
  // Likewise on inputs A and B
  ok = ok && command(0, kTV1WindowIDA, kTV1FunctionAdjustSourceHDCPAdvertize, state);
  ok = ok && command(0, kTV1WindowIDB, kTV1FunctionAdjustSourceHDCPAdvertize, state);
  ok = ok && command(0, kTV1WindowIDA, kTV1FunctionAdjustSourceHDCPStatus, state);
  ok = ok && command(0, kTV1WindowIDB, kTV1FunctionAdjustSourceHDCPStatus, state);
  
  return ok;
}

bool SPKTVOne::set1920x480(int resStoreNumber) 
{
  bool ok;

  ok = command(0, 0, kTV1FunctionAdjustResolutionImageToAdjust, resStoreNumber);
  if (ok)
  {
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionInterlaced, 0);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionFreqCoarseH, 31475);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionFreqFineH, 31475);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionActiveH, 1920);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionActiveV, 480);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionStartH, 240); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionStartV, 5); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionCLKS, 2400); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionLines, 525);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncH, 192);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncV, 30); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncPolarity, 0);
  }
  
  return ok;
}

bool SPKTVOne::set1600x600(int resStoreNumber) 
{
  bool ok;

  ok = command(0, 0, kTV1FunctionAdjustResolutionImageToAdjust, resStoreNumber);
  if (ok)
  {
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionInterlaced, 0);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionFreqCoarseH, 37879);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionFreqFineH, 37879);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionActiveH, 1600);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionActiveV, 600);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionStartH, 192); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionStartV, 14); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionCLKS, 2112); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionLines, 628);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncH, 160);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncV, 13); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncPolarity, 0);
  }
    
  return ok;
}

bool SPKTVOne::set2048x768(int resStoreNumber) 
{
  bool ok;
  
  ok = command(0, 0, kTV1FunctionAdjustResolutionImageToAdjust, resStoreNumber);
  if (ok)
  {
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionInterlaced, 0);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionFreqCoarseH, 48363);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionFreqFineH, 48363);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionActiveH, 2048);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionActiveV, 768);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionStartH, 224); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionStartV, 11); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionCLKS, 2688); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionLines, 806);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncH, 368);
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncV, 24); 
      ok = ok && command(0, 0, kTV1FunctionAdjustResolutionSyncPolarity, 0);
  }
    
  return ok;
}

void SPKTVOne::signErrorOff() {
    *errorDO = 0;
}

bool SPKTVOne::uploadEDID(char* edidData, int edidDataLength, int edidSlotIndex)
{
    // TASK: Upload EDID

    // This command is reverse engineered from a VB snippet and RS232 comms logged between TVOne test app and unit 
    
    // To write EDID, its broken into chunks and sent as a series of extra-long commands
    // Command: 8 bytes of command (see code below) + 32 bytes of EDID payload + End byte
    // Acknowledgement: 53 02 40 95 (Hex)
    
    if (debug) debug->printf("Upload EDID, length %i to index %i \r\n", edidDataLength, edidSlotIndex);
    
    bool success = false;

    int commandLength = 8+32+1;
    int ackLength = 4;
    char goodAck[] = {0x53, 0x02, 0x40, 0x95};
    
    // We want to upload full EDID slot, ie. zero out to 256 even if edidData is only 128bytes.
    for (int i=0; i<256; i=i+32)
    {
        char command[commandLength];

        command[0] = 0x53;
        command[1] = 0x27;
        command[2] = 0x22;
        command[3] = 0x7;
        command[4] = edidSlotIndex;
        command[5] = 0;
        command[6] = i / 32; // ie. chunk index
        command[7] = 0;

        for (int j=0; j<32; j++)
        {
          if (i+j < edidDataLength) 
            *(command+8+j) = edidData[i+j];
          else 
            *(command+8+j) = 0;
        }

        command[8+32] = 0x3F;

        if (debug)
        {
            debug->printf("EDID command: ");
            for (int k=0; k < commandLength; k++) debug->printf(" %x", command[k]);
            debug->printf("\r\n");
        }

        while (serial->readable() || timer.read_ms() < 1000) 
        {
           if (serial->readable()) serial->getc();
        }
 
        for (int k=0; k < commandLength; k++) serial->putc(command[k]);
        
        timer.reset();
        
        char ackBuffer[4];
        int  ackPos = 0;
        while (timer.read_ms() < 1000) 
        {
            if (serial->readable()) ackBuffer[ackPos++] = serial->getc();
            if (ackPos == 4) break;
        }
          
        if (memcmp(ackBuffer, goodAck, ackLength) == 0) 
        {
            success = true;
        }
        else
        {
            success = false;
            if (debug) 
            {
                debug->printf("EDID Part write failed. Ack:");
                for (int k = 0; k < ackLength; k++) debug->printf(" %x", ackBuffer[k]);
                debug->printf("\r\n");
            }
            break;
        }
    }
    
    return success;
}
