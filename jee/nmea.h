// Driver for an NMEA GPS, tested with GlobalTop PA6H
// Ultra-simple NMEA parsing adapted from http://www.technoblogy.com/show?SJ0

// GlobalTop PA6H RMC stanza
// Ex: $GPRMC,194509.000,A,4042.6142,N,07400.4168,W,2.03,5.84,160412,,,A*77
static char nmea_gprmc[]="$GPRMC,dddtdd.ddm,A,eeae.eeee,l,eeeae.eeee,o,djdk,djdc,eeeeey,,,?*??";

struct NMEA {
    uint8_t state = 0;
    uint16_t utemp;
    int32_t stemp;

    // GPS variables
    uint16_t time, msecs; // time as in 1945 == 19:45
    uint16_t knots, course; // knots*100, degrees*100
    uint32_t date; // date as in 123117 for december 31st 2017
    int32_t lat, lon;
    bool fix;

    bool parse(char c) {
      if (c == '$') { state = 0; utemp = 0; stemp = 0;}
      else if (state == 0) return false;
      char mode = nmea_gprmc[state++];
      // If received character matches format string, or format is '?' - return
      if ((mode == c) || (mode == '?')) return false;
      // d=decimal digit
      char d = c - '0';
      if (mode == 'd') utemp = utemp*10 + d;
      // e=long decimal digit
      else if (mode == 'e') stemp = stemp*10 + d;
      // a=angular measure
      else if (mode == 'a') stemp = stemp*6 + d;
      // t=Time - hhmm
      else if (mode == 't') { time = utemp*10 + d; utemp = 0; }
      // m=Millisecs
      else if (mode == 'm') { msecs = utemp*10 + d; utemp = 0; }
      // l=Latitude - in minutes*1000
      else if (mode == 'l') { if (c == 'N') lat = stemp; else lat = -stemp; stemp = 0; }
      // o=Longitude - in minutes*1000
      else if (mode == 'o') { if (c == 'E') lon = stemp; else lon = -stemp; stemp = 0; }
       // j/k=Speed - in knots*100
      else if (mode == 'j') { if (c != '.') { utemp = utemp*10 + d; state--; } }
      else if (mode == 'k') { knots = utemp*10 + d; utemp = 0; }
      // c=Course (Track) - in degrees*100
      else if (mode == 'c') { course = utemp*10 + d; utemp = 0; }
      // y=Date - ddmmyy
      else if (mode == 'y') { date = stemp*10 + d ; fix = 1; return true; }
      else state = 0;
      return false;
    }
};
