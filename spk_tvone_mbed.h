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

#ifndef SPKTVOne_mBed_h
#define SPKTVOne_mBed_h

#include "spk_tvone.h"
#include "mbed.h"

class SPKTVOne
{
  public:
    SPKTVOne(PinName txPin, PinName rxPin, PinName signWritePin = NC, PinName signErrorPin = NC, Serial *debugSerial = NULL);
    
    enum commandType {writeCommandType = 0, readCommandType = 1};
    static const int standardAckLength = 20;
    
    bool command(uint8_t channel, uint8_t window, int32_t func, int32_t payload);
    bool readCommand(uint8_t channel, uint8_t window, int32_t func, int32_t &payload);
    
    bool uploadEDID(char* edidData, int edidDataLength, int edidSlotIndex);
        
    bool setCustomResolutions();
    bool setHDCPOn(bool state);
    
    int timeoutCommandPeriod;
    int minimumCommandPeriod;

    int  millisSinceLastCommandSent();
     
  private:
    bool command(commandType readWrite, int* ackBuffer, int ackLength, uint8_t channel, uint8_t window, int32_t func, int32_t payload);
    
    bool set1920x480(int resStoreNumber);
    bool set1600x600(int resStoreNumber);
    bool set2048x768(int resStoreNumber);
    
    Serial *serial;
    Serial *debug; 
    
    Timer timer;
    
    DigitalOut *writeDO;
    DigitalOut *errorDO;
    Timeout signErrorTimeout;
    void signErrorOff();
};

#endif
