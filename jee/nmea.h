// Driver for an NMEA GPS, tested with GlobalTop PA6H
// Ultra-simple NMEA parsing adapted from http://www.technoblogy.com/show?SJ0

// GlobalTop PA6H RMC stanza
// Ex: $GPRMC,194509.000,A,4042.6142,N,07400.4168,W,2.03,5.84,160412,,,A*77
//static char nmea_gprmc[]="$GPRMC,dddtdd.ddm,A,eeae.eeee,l,eeeae.eeee,o,djdk,djdc,eeeeey,,,?*??";

static struct { char *name; char *fmt; }
    nmea_stanzas[] = {
        { "GPRMC", "dddtdd.ddm,A,eeae.eeee,l,eeeae.eeee,o,djdk,djdc,eeeeey,,,?" },
        { "GPGGA", "??????????,?????????,?,??????????,?,?,s,d.dh,efA,M,x" },
        { 0, 0 },
    };
static int nmea_fix = 0;

int printf(const char* fmt, ...);

struct NMEA {
    uint8_t state = 0; // 0=looking for $, 1=accum format, 2=parsing data
    uint8_t pos = 0; // position in format
    uint8_t stanza; // index into nmea_stanzas for the one we're currently parsing
    char fmt[7]; // accumulator for format parsing
    uint16_t utemp;
    int32_t stemp;

    // GPS variables
    uint16_t time, msecs; // time split into HHMM and SS.SSS
    uint16_t knots, course; // knots*100, degrees*100
    uint32_t date; // date as in 123117 for december 31st 2017
    uint16_t hdop; // HDOP*100
    uint16_t sats; // number of satellites used in fix
    int32_t lat, lon; // minutes*1E3 (*5/3 converts to degrees*1E6)
    int32_t alt; // decimeters
    bool fix;

    // string equality
    bool streq(char s1[], char s2[]) {
        while (true) {
            if (!*s1) return !*s2;
            if (!*s2) return false;
            if (*s1++ != *s2++) return false;
        }
    }

    bool parse(char c) {
        switch (state) {
        case 0: // wait for '$'
            if (c == '$') { state++; pos = 0; } // found start of stanza
            return false;
        case 1: // accumlate stanza format name, e.g. GPRMC
            if (c != ',') {
                if (pos > 5) state = 0; // format too long
                else fmt[pos++] = c; // accumulate format name
                return false;
            }
            fmt[pos++] = 0; // null terminate string
            // see whether we have a format to parse this stanza
            for (int i=0; i<2; i++) {
                if (streq(nmea_stanzas[i].name, fmt)) {
                    stanza = i;
                    state++;
                    pos = 0; utemp = 0; stemp = 0;
                    return false;
                }
            }
            state = 0; // unknown stanza
            return false;
        case 2: {
            if (c == '*') {
                if (stanza == nmea_fix) { fix = 1; return true; }
                state = 0; return false;
            }
            char mode = nmea_stanzas[stanza].fmt[pos++];
            //printf("%c", mode);
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
            else if (mode == 'j') { if (c != '.') { utemp = utemp*10 + d; pos--; } }
            else if (mode == 'k') { knots = utemp*10 + d; utemp = 0; }
            // c=Course (Track) - in degrees*100
            else if (mode == 'c') { course = utemp*10 + d; utemp = 0; }
            // y=Date - ddmmyy
            else if (mode == 'y') { date = stemp*10 + d ; }
            // s=Satellites
            else if (mode == 's') { sats = d; }
            // h=HDOP
            else if (mode == 'h') { hdop = utemp*10 + d; }
            // A=Altitude in decimeters
            else if (mode == 'f') { if (c != '.') { stemp = stemp*10 + d; pos--; } }
            else if (mode == 'A') { alt = stemp*10 + d; }
            // x=any field contents
            else if (mode == 'x') { if (c != nmea_stanzas[stanza].fmt[pos]) pos--; else pos++;}
            else state = 0;
            return false;
            }
        default:
            state = 0;
            return false;
        }
    }
};
