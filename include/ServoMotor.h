#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

class Servo360Motor {
private:
    int servoPin;
    Servo myServo;
    
    // Variabel untuk estimasi sudut (karena servo 360 tidak punya sensor posisi)
    float currentAngle; 
    float speedDegreesPerSecCW;   // Kecepatan saat putar CW (deg/s)
    float speedDegreesPerSecCCW;  // Kecepatan saat putar CCW (deg/s)

public:
    // Constructor dengan konfigurasi default (diambil dari main.cpp sebelumnya)
    Servo360Motor(int pin = 21, float speedCW = 55.0, float speedCCW = 45.0) {
        servoPin = pin;
        currentAngle = 0.0;
        speedDegreesPerSecCW  = speedCW;   // Hasil kalibrasi CW
        speedDegreesPerSecCCW = speedCCW;  // Hasil kalibrasi CCW
    }
    
    void begin() {
        // Alokasi timer wajib untuk ESP32Servo
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);

        myServo.setPeriodHertz(50);
        // Range 500-2400us adalah standar umum untuk MG996R
        myServo.attach(servoPin, 500, 2400);
        stop();
    }
    
    // Putar penuh searah jarum jam (Clockwise)
    void rotateCW() {
        myServo.write(0);
    }

    // Putar penuh berlawanan arah jarum jam (Counter-Clockwise)
    void rotateCCW() {
        myServo.write(180);
    }

    // Berhenti
    void stop() {
        // Nilai 90 adalah titik berhenti standar servo 360.
        // Jika servo masih mendengung/berputar sangat pelan, ubah ke 89 atau 91.
        myServo.write(90); 
    }
    
    // Fungsi pengganti "step" pada Stepper Motor. 
    // Menggunakan delay waktu untuk mengestimasi pergerakan sudut.
    // PENTING: Metode ini tidak akan seakurat Stepper Motor asli.
    void moveAngle(float targetAngleDelta, bool isCW) {
        float speed = isCW ? speedDegreesPerSecCW : speedDegreesPerSecCCW;
        float timeToMoveMs = (targetAngleDelta / speed) * 1000.0;
        
        if (isCW) {
            rotateCW();
            currentAngle += targetAngleDelta;
        } else {
            rotateCCW();
            currentAngle -= targetAngleDelta;
        }
        
        delay((int)timeToMoveMs);
        stop();
        
        if (currentAngle >= 360.0) currentAngle = fmod(currentAngle, 360.0);
        if (currentAngle < 0.0)   currentAngle = 360.0 + fmod(currentAngle, 360.0);
    }
    
    float getAngle() {
        return currentAngle;
    }

    void resetAngle() {
        currentAngle = 0.0;
    }
};

#endif // SERVO_MOTOR_H
