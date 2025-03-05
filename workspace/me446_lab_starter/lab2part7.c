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
long timestep = 0;

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
float theta1_desired = 0;
float theta2_desired = 0;
float theta3_desired = 0;
float Theta1_old = 0;
float Omega1_raw = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;
float Theta2_old = 0;
float Omega2_raw = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;
float Theta3_old = 0;
float Omega3_raw = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;
float error1 = 0;
float error2 = 0;
float error3 = 0;


float Kp1 = 55.0, Kd1 = 2.7, Ki1 = 500;
float Kp2 = 55.0, Kd2 = 5, Ki2 = 500;
float Kp3 = 55.0, Kd3 = 4, Ki3 = 500;
float integral1 = 0,integral2 = 0,integral3 = 0;
float threshold = 0.015;
float J1 = 0.0167, J2=0.03,J3 = 0.0128;
float theta1_dot_desired = 0, theta2_dot_desired = 0, theta3_dot_desired = 0;
float theta1_ddot_desired = 0, theta2_ddot_desired = 0, theta3_ddot_desired = 0;

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



//    if ((mycount%5000)==0){
//        if (theta1_desired>0.25){
//            theta1_desired = 0;
//            theta2_desired = 0;
//            theta3_desired = 0;
//        }
//        else {
//            theta1_desired = 0.5;
//            theta2_desired = 0.5;
//            theta3_desired = 0.5;
//        }
//
//    }
    float time_trajectory = (mycount % 2000); // 0 to 2 seconds



    //Motor torque limitation(Max: 5 Min: -5)
    Omega1_raw = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;


    Omega2_raw = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;


    Omega3_raw = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0;
//
//
    error1 = theta1_desired - theta1motor;
    error2 = theta2_desired - theta2motor;
    error3 = theta3_desired - theta3motor;

    if (fabs(error1) < threshold) {
        integral1 += (error1 + (theta1_desired - Theta1_old)) * 0.0005; // Trapezoidal rule
    } else {
        integral1 = 0;
    }

    if (fabs(error2) < threshold) {
        integral2 += (error2 + (theta2_desired - Theta2_old)) * 0.0005;
    } else {
        integral2 = 0;
    }

    if (fabs(error3) < threshold) {
        integral3 += (error3 + (theta3_desired - Theta3_old)) * 0.0005;
    } else {
        integral3 = 0;
    }

    *tau1 = (J1 * theta1_ddot_desired)
             + (Kp1 * (theta1_desired - theta1motor))
             + (Kd1 * (theta1_dot_desired - Omega1))
             + (Ki1 * integral1);

    *tau2 = (J2 * theta2_ddot_desired)
                     + (Kp2 * (theta2_desired - theta2motor))
                     + (Kd2 * (theta2_dot_desired - Omega2))
                     + (Ki2 * integral2);
    *tau3 = (J3 * theta3_ddot_desired)
                     + (Kp3 * (theta3_desired - theta3motor))
                     + (Kd3 * (theta3_dot_desired - Omega3))
                     + (Ki3 * integral3);

    if (*tau1 > 5.0) *tau1 = 5.0;
    if (*tau1 < -5.0) *tau1 = -5.0;
    if (*tau2 > 5.0) *tau2 = 5.0;
    if (*tau2 < -5.0) *tau2 = -5.0;
    if (*tau3 > 5.0) *tau3 = 5.0;
    if (*tau3 < -5.0) *tau3 = -5.0;

    Theta1_old = theta1motor;
    //order matters here. Why??
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Theta2_old = theta2motor;
    //order matters here. Why??
    Omega2_old2 = Omega1_old2;
    Omega2_old1 = Omega2;

    Theta3_old = theta3motor;
    //order matters here. Why??
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


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
    Simulink_PlotVar1 = error1;
    Simulink_PlotVar2 = error2;
    Simulink_PlotVar3 = error3;
    Simulink_PlotVar4 = theta1_desired;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "printmotor: %.2f,%.2f,%.2f, position: %.2f,%.2f,%.2f, \n\r cal_motor %.2f,%.2f,%.2f, dh_motor %.2f,%.2f,%.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x_end,y_end,z_end,cal_theta1motor*180/PI,cal_theta2motor*180/PI,cal_theta3motor*180/PI,cal_theta1*180/PI,cal_theta2*180/PI,cal_theta3*180/PI);
    } else {
        serial_printf(&SerialA, "whattoprintout   \n\r");
    }
}

