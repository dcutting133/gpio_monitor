#include "ros/ros.h"
#include "wiringPi.h"
#include "gpio_monitor/gpio_input.h"
#include "gpio_monitor/gpio_output.h"

/**
 * RASPBERRY PI 3B PINS
 * 
 *  Pin values for each given component controlled using GPIO.
 * 
 * https://www.raspberrypi.org/documentation/usage/gpio-plus-and-raspi2/
 */

#define INPUT1 0 // (pin 17 / phys pin 11), ESTOP
#define INPUT2 2 // (pin 27 / phys pin 13), NA
#define INPUT3 3 // (pin 22 / phys pin 15), NA

#define OUTPUT1 1 // (pin 18 / phys pin 12), drive mode led
#define OUTPUT2 4 // (pin 23 / phys pin 16), NA
#define OUTPUT3 5 // (pin 24 / phys pin 18), GPIO initialized

gpio_monitor::gpio_input inputs;
bool input1;
bool input2;
bool input3;

gpio_monitor::gpio_output outputs;
bool output1;
bool output2;
bool output3;

ros::Publisher gpioPub;

void initPins() {
    pinMode(INPUT1, INPUT);
    pinMode(INPUT2, INPUT);
    pinMode(INPUT3, INPUT);
    pinMode(OUTPUT1, OUTPUT);
    pinMode(OUTPUT2, OUTPUT);
    pinMode(OUTPUT3, OUTPUT);
    input1 = digitalRead(INPUT1) == 0 ? false : true;
    input2 = digitalRead(INPUT2) == 0 ? false : true;
    output1 = false;
    output2 = false;
    if(ros::ok()) {
        output3 = true;
    } else {
        output3 = false;
    }
}

void checkInputs() {
    input1 = digitalRead(INPUT1) == 0 ? false : true;
    input2 = digitalRead(INPUT2) == 0 ? false : true;
    input3 = digitalRead(INPUT3) == 0 ? false : true;
    inputs.input1 = input1;
    inputs.input2 = input2;
    inputs.input3 = input3;
    gpioPub.publish(inputs);
}

void outputCallback(const gpio_monitor::gpio_output::ConstPtr& outputs) {
    output1 = outputs->output1;
    output2 = outputs->output2;
    if (output1) {
        digitalWrite(OUTPUT1, HIGH);
    } else {
        digitalWrite(OUTPUT1, LOW);
    }

    if (output2) {
        digitalWrite(OUTPUT2, HIGH);
    } else {
        digitalWrite(OUTPUT2, LOW);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gpio_monitor");
    ros::NodeHandle n;
    wiringPiSetup(); // Enables WiringPi for GPIO control
    initPins();

    if (output3) {
        digitalWrite(OUTPUT3, HIGH);
    } else {
        digitalWrite(OUTPUT3, LOW);
    }

    ros::Subscriber gpioSub = n.subscribe("gpio/outputs", 5, outputCallback);
    gpioPub = n.advertise<gpio_monitor::gpio_input>("gpio/inputs", 5);
    checkInputs();
    ros::spin();
    return 0;
}
