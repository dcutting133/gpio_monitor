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
#define OUTPUT3 5 // (pin 24 / phys pin 18), NA

gpio_monitor::gpio_input inputs;
bool input1;

gpio_monitor::gpio_output outputs;
bool output1;

ros::Publisher gpioPub;

void initPins() {
    pinMode(INPUT1, INPUT);
    pinMode(OUTPUT1, OUTPUT);
}

void updateOutputs() {

}

void outputCallback(const gpio_monitor::gpio_output::ConstPtr& outputs) {
    output1 = outputs->driveMode;
    updateOutputs();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gpio_monitor");
    ros::NodeHandle n;
    wiringPiSetup(); // Enables WiringPi for GPIO control
    initPins();

    digitalWrite(OUTPUT1, HIGH);
    ros::Subscriber gpioSub = n.subscribe("gpio/outputs", 5, outputCallback);
    gpioPub = n.advertise<gpio_monitor::gpio_input>("gpio/inputs", 5);
    ros::spin();
    return 0;
}
