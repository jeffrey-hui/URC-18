#include <Arduino.h>
#include <Wire.h>

class GPSTracker {
public:
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
    uint16_t len;
    unsigned char ** strs;
};

GPSTracker gps;
const int ADDRESS = 0x48;

void onRequest() {
    if (gps.ok()) {
        uint8_t length = gps.topLen();
        Wire.write(0x01);
        Wire.write(length);
        unsigned char * stringy_the_beef = gps.popStr();
        Wire.write(reinterpret_cast<uint8_t *>(stringy_the_beef), length);
        free(stringy_the_beef);
    }
    else {
        Wire.write(0x00);
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
    auto * buffer = reinterpret_cast<unsigned char *>(malloc(bufLen));
    amt.getBytes(buffer, bufLen);
    gps.recordDatum(bufLen, buffer);
}