#include <Arduino.h>
#include <Wire.h>

class GPSTracker {
public:
    GPSTracker() {
        len = 0;
        datas = reinterpret_cast<uint8_t *>(malloc(0));
        strs = reinterpret_cast<unsigned char **>(malloc(0));
    }

    void recordDatum(uint8_t dlen, unsigned char * d) {
        push();
        *(datas + (len - 1)) = dlen;
        *(strs + (len - 1)) = d;
    }

    uint8_t topLen() {
        return *(datas + (len - 1));
    }

    unsigned char * popStr() {
        unsigned char * top = *(strs + (len - 1));
        pop();
        return top;
    }

    bool ok() {
        return len > 0;
    }
private:
    void push() {
        len+=1;
        datas = reinterpret_cast<uint8_t *>(realloc(datas, len * sizeof(uint8_t)));
        strs = reinterpret_cast<unsigned char **>(realloc(strs, len*sizeof(unsigned char *)));
    }

    void pop() {
        len-=1;
        datas = reinterpret_cast<uint8_t *>(realloc(datas, len * sizeof(uint8_t)));
        strs = reinterpret_cast<unsigned char **>(realloc(strs, len*sizeof(unsigned char *)));
    }

    uint8_t *datas;
    uint16_t len = 0;
    unsigned char ** strs;
};

GPSTracker gps;
const int ADDRESS = 0x48;

int context = 0;

void onRequest() {
    if (gps.ok()) {
        if (context == 0) {
            Wire.write(0x01);
            context += 1;
        }
        else if (context == 1) {
            uint8_t length = gps.topLen();
            Wire.write(length);
            context += 1;
        }
        else if (context == 2) {
            uint8_t length = gps.topLen();
            unsigned char *stringy_the_beef = gps.popStr();
            Wire.write(reinterpret_cast<uint8_t *>(stringy_the_beef), length);
            free(stringy_the_beef);
            context = 0;
        }
    }
    else {
        Wire.write(0x00);
        context = 0;
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin(ADDRESS);
    Wire.onRequest(onRequest);
}

void loop() {
    String amt = Serial.readStringUntil('\n');
    uint8_t bufLen = amt.length();
    if (bufLen < 3) return;
    char * buffer = reinterpret_cast<char *>(malloc(bufLen));
    amt.toCharArray(buffer, bufLen);
    gps.recordDatum(bufLen, reinterpret_cast<unsigned char *>(buffer));
}
