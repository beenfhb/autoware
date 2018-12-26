//============================================================================
// Name        : ReadNMEASentence.h
// Author      : pino
// Version     :
// Copyright   : free
// Description :
//============================================================================

#ifndef __READNMEASENTENCE__H_
#define __READNMEASENTENCE__H_

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <string>

namespace NMEA_PARSER {

using namespace std;
typedef unsigned int uint;
typedef unsigned char uchar;

enum parseStatus { waitFirstChar = 0, command, data, checkSum1, checkSum2 };

class ReadNMEASentence {
private:
  void GxRMC();
  void GPGGA();
  void HEHDT();

  parseStatus Stat;
  string nmeaCmd;
  string dataString;
  char receivedCheckSum;
  char calculatedCheckSum;

public:
  // GxRMC
  struct {
    char status;
    double latitude;
    double longitude;
    double groundSpeed;
    double direction;
    double magVariation;
    int year;
    int month;
    int day;
    int hour;
    int min;
    int sec;
    uint count;
  } gxrmc;

  // GPGGA
  struct {
    int hour;
    int min;
    int sec;
    double latitude;
    double longitude;
    char mode;
    int satellites;
    double hdop;
    double altitude;
    uint count;
  } gpgga;

  // HEHDT
  struct {
    double trueHeading;
    uint count;
  } hehdt;

  uint commandCount;
  string GetField(const string dataString, uint *pos);
  void Parse(char letter);
  ReadNMEASentence();
  ~ReadNMEASentence();
};

} // namespace NMEA_PARSER

#endif /* __READNMEASENTENCE__H_ */
