#include "hardware/pwm.h"

#define ESC_CALIB_LOW 1000
#define ESC_CALIB_HIGH 2000
#define MIN_PULSE 1000
#define MAX_PULSE 1800
#define STOP_PULSE 1000
#define IDLE_PULSE 1060

#define PIN10_SLICE 5
#define PIN12_SLICE 6

// GP10 CW              CCW GP11
//          O       O
//              O
//          O       O
// GP13 CCW             CW  GP12

// variables for RC receiver channels
uint16_t chPitch, chRoll, chThrottle, chYaw = 0;

// variables for ESCs
int motor10, motor11, motor12, motor13 = STOP_PULSE;
byte data = 0;

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);
    while (!Serial); // we need this, otherwise the sketch will start without waiting for the serial monitor
    
    setupPWM();

    // arm the ECSs
    writeESCs(STOP_PULSE);
    delay(500);

    Serial.print("Press c to calibrate, 1-4 to test single motor, 5 to test all motors");
}

int pulse = MIN_PULSE;

void loop() {
     if (Serial.available()) {
        data = Serial.read();
        delay(100);                   
        while(Serial.available() > 0) Serial.read();     
    }

    if (data == 'c') {
        calibrateESCs();
        data = 0;
    }

    decodeIBus();  

    pulse = chThrottle;
    pulse = constrain(pulse, IDLE_PULSE, MAX_PULSE);
    Serial.print(F("Pulse:"));
    Serial.println(pulse);

    motor10 = STOP_PULSE;
    motor11 = STOP_PULSE;
    motor12= STOP_PULSE;
    motor13 = STOP_PULSE;

    if (data == '1' || data == '5') {
        motor10 = pulse;                   
        Serial.println("Motor10 ON - CW - Front Left");
    }

    if (data == '2' || data == '5') {
        motor11 = pulse;                   
        Serial.println("Motor11 ON - CCW - Front Right");
    }

    if (data == '3' || data == '5') {
        motor12 = pulse;                   
        Serial.println("Motor12 ON - CW - Rear Right");
    }
    
    if (data == '4' || data == '5') {
        motor13 = pulse;                   
        Serial.println("Motor13 ON - CCW - Rear Left");
    }
    
    sendToESCs();
    delay(200);
}

void setupPWM() {
    // PWM 5A - pin14 - GPIO10 - slice 5 - CHAN A
    // PWM 5B - pin15 - GPIO11 - slice 5 - CHAN B
    // PWM 6A - pin16 - GPIO12 - slice 6 - CHAN A
    // PWM 6B - pin17 - GPIO13 - slice 6 - CHAN B
    gpio_set_function(10, GPIO_FUNC_PWM);
    gpio_set_function(11, GPIO_FUNC_PWM);
    gpio_set_function(12, GPIO_FUNC_PWM);
    gpio_set_function(13, GPIO_FUNC_PWM);
    
    // clock is 125Mhz, so 1 tick is 8 nanoseconds. I want a cycle of 1 micro, so the clock divider is 125
    pwm_set_clkdiv(PIN10_SLICE, 125);
    pwm_set_clkdiv(PIN12_SLICE, 125);
    
    // this is the maximum counter (16 bit 0-65535). set period of 2040 micros, 490.19 Hz
    pwm_set_wrap(PIN10_SLICE, 2039);
    pwm_set_wrap(PIN12_SLICE, 2039);
    
    pwm_set_chan_level(PIN10_SLICE, PWM_CHAN_A, STOP_PULSE); // GPIO10
    pwm_set_chan_level(PIN10_SLICE, PWM_CHAN_B, STOP_PULSE); // GPIO11
    pwm_set_chan_level(PIN12_SLICE, PWM_CHAN_A, STOP_PULSE); // GPIO12
    pwm_set_chan_level(PIN12_SLICE, PWM_CHAN_B, STOP_PULSE); // GPIO13
    
    pwm_set_enabled(PIN10_SLICE, true);
    pwm_set_enabled(PIN12_SLICE, true);
}

void calibrateESCs() {
    Serial.println(F("ESC calibration, check that battery IS DISCONNECTED and press a key"));
    while(!Serial.available());
    Serial.read();
    writeESCs(ESC_CALIB_HIGH);
    Serial.println(F("MAX pulse sent, press a key after the ESC has emitted 3 beeps"));
    while(!Serial.available());
    Serial.read();
    writeESCs(ESC_CALIB_LOW);
    Serial.println(F("ESC calibrated"));
    delay(5000);
}

void writeESCs(int micros) {
    // send same pulse to all 4 ESCs
    motor10 = micros;
    motor11 = micros;
    motor12 = micros;
    motor13 = micros;

    sendToESCs();
}

void sendToESCs() {
    pwm_set_chan_level(PIN10_SLICE, PWM_CHAN_A, motor10);
    pwm_set_chan_level(PIN10_SLICE, PWM_CHAN_B, motor11);
    pwm_set_chan_level(PIN12_SLICE, PWM_CHAN_A, motor12); 
    pwm_set_chan_level(PIN12_SLICE, PWM_CHAN_B, motor13);
}

#define IBUS_BUFFER_SIZE 32
static uint8_t buffer[IBUS_BUFFER_SIZE];
static uint8_t bufferIndex = 0;
uint16_t checksum = 0;

void decodeIBus() {
    // lenght: 0x20
    // Command: 0x40
    // ch1 LSB+MSB
    // ..
    // ch14 LSB-MSB
    // checksum
    while (Serial1.available() > 0) {
        uint8_t value = Serial1.read();
        if (bufferIndex == 0 && value != 0x20) {continue;}
        if (bufferIndex == 1 && value != 0x40) {bufferIndex = 0; continue;}
        if (bufferIndex < IBUS_BUFFER_SIZE) buffer[bufferIndex] = value;
        bufferIndex++;

        if (bufferIndex == IBUS_BUFFER_SIZE) {
            bufferIndex = 0;
            uint16_t expected = buffer[IBUS_BUFFER_SIZE-1] * 256 + buffer[IBUS_BUFFER_SIZE-2];
            checksum =0;
            for (int i=0; i < IBUS_BUFFER_SIZE-2; i++) {checksum+=buffer[i];}
            if ((expected + checksum) == 0xFFFF) {
                if (buffer[1] == 0x40) { 
                    chRoll = buffer[3] * 256 + buffer[2];
                    chPitch = buffer[5] * 256 + buffer[4];
                    chThrottle = buffer[7] * 256 + buffer[6];
                    chYaw = buffer[9] * 256 + buffer[8];
                }
            }
        }
    }  
}