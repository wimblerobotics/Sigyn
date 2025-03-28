#include <WiringPi.h>
#include <softPwm.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    // Initialize the wiringPi library
    wiringPiSetup();
    // Set the pin mode to PWM output
    // pinMode(1, PWM_OUTPUT);
    // // Set the PWM frequency to 50Hz
    // pwmSetMode(PWM_MODE_MS);
    // pwmSetClock(384);
    // pwmSetRange(1000);

    // // Set the PWM duty cycle to 50%
    // pwmWrite(1, 50);

    // // Wait for 5 seconds
    // delay(5000);

    // // Set the PWM duty cycle to 0%
    // pwmWrite(1, 0);
    printf("Hello, world!\n");

    return 0;
}