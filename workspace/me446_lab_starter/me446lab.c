#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.403; //-0.37;
float offset_Enc3_rad = 0.236; //0.27;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;
#pragma DATA_SECTION(what2print, ".my_vars")
float what2print = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;
float cal_theta1 = 0;
float cal_theta2 = 0;
float cal_theta3 = 0;
float cal_theta1motor = 0;
float cal_theta2motor = 0;
float cal_theta3motor = 0;
float sigma1 = 0;
// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;
float x_end = 0;
float y_end = 0;
float z_end = 0;

void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
}




// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;
    x_end = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y_end = 0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z_end = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;
    sigma1 = sqrt(x_end * x_end + y_end * y_end + (0.254 - z_end) * (0.254 - z_end));
    cal_theta1 = atan2(y_end, x_end);
    cal_theta2 = -(acos((2 * x_end * x_end + 2 * y_end * y_end) / (2 * sqrt(x_end * x_end + y_end * y_end) * sigma1)) + acos((x_end * x_end + y_end * y_end + (0.254 - z_end) * (0.254 - z_end)) / (2 * 0.254 * sigma1)));
    cal_theta3 = acos((x_end * x_end - 2*0.254 * 0.254 + y_end * y_end + (0.254 - z_end) * (0.254 - z_end)) / (2 * 0.254 * 0.254));
    cal_theta1motor = cal_theta1;
    cal_theta2motor = cal_theta2 + PI/2;
    cal_theta3motor = cal_theta3 + cal_theta2;
    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = 0;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "printmotor: %.2f,%.2f,%.2f, position: %.2f,%.2f,%.2f, \n\r cal_motor %.2f,%.2f,%.2f, dh_motor %.2f,%.2f,%.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x_end,y_end,z_end,cal_theta1motor*180/PI,cal_theta2motor*180/PI,cal_theta3motor*180/PI,cal_theta1*180/PI,cal_theta2*180/PI,cal_theta3*180/PI);
    } else {
        serial_printf(&SerialA, "whattoprintout   \n\r");
    }
}

