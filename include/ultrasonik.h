#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

#define NUM_SENSOR 8

class Sensor {
private:
    int trigPins[NUM_SENSOR];
    int echoPins[NUM_SENSOR];
    float distanceVal[NUM_SENSOR];

    float readSingleUltrasonic(int trigPin, int echoPin) {
        // trigger
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // tunggu echo HIGH
        long start = micros();
        while (digitalRead(echoPin) == LOW) {
            if (micros() - start > 24000) return -1;
        }
        long echoStart = micros();

        // tunggu echo LOW
        while (digitalRead(echoPin) == HIGH) {
            if (micros() - echoStart > 24000) return -1;
        }

        long duration = micros() - echoStart;
        float dist = duration * 0.0343 / 2.0;

        // filter kasar (noise)
        if (dist < 2 || dist > 400) return -1;

        return dist;
    }

public:
    Sensor(const int trig[], const int echo[]) {
        for (int i = 0; i < NUM_SENSOR; i++) {
            trigPins[i] = trig[i];
            echoPins[i] = echo[i];
            distanceVal[i] = 0.0;
        }
    }

    // Constructor default dengan konfigurasi pin langsung (diambil dari main.cpp)
    Sensor() {
        const int defaultTrig[NUM_SENSOR] = {19, 5, 4, 23, 22, 25, 27, 12};
        const int defaultEcho[NUM_SENSOR] = {18, 35, 34, 32, 33, 26, 14, 13};
        for (int i = 0; i < NUM_SENSOR; i++) {
            trigPins[i] = defaultTrig[i];
            echoPins[i] = defaultEcho[i];
            distanceVal[i] = 0.0;
        }
    }

    void begin() {
        for (int i = 0; i < NUM_SENSOR; i++) {
            pinMode(trigPins[i], OUTPUT);
            pinMode(echoPins[i], INPUT);
            digitalWrite(trigPins[i], LOW);
        }
    }

    void readAll() {
        for (int i = 0; i < NUM_SENSOR; i++) {
            distanceVal[i] = readSingleUltrasonic(trigPins[i], echoPins[i]);
            delay(20);  // Mengembalikan delay ke 20ms untuk mencegah sinyal suara saling bertabrakan (crosstalk)
        }
    }

    // Fungsi tambahan untuk membaca hanya satu sensor tertentu
    float read(int index) {
        if (index >= 0 && index < NUM_SENSOR) {
            distanceVal[index] = readSingleUltrasonic(trigPins[index], echoPins[index]);
            return distanceVal[index];
        }
        return -1.0;
    }

    float getDistance(int index) {
        if (index >= 0 && index < NUM_SENSOR) {
            return distanceVal[index];
        }
        return -1.0;
    }
};

#endif // SENSOR_H
