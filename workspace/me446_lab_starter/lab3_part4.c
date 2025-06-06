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

// Variables used for Tera Term
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

// End effector positions (Forward Kinematics)
float x_end = 0;
float y_end = 0;
float z_end = 0;

// Desired joint angles
float theta1_desired = 0;
float theta2_desired = 0;
float theta3_desired = 0;

// Joint angles used in velocity tracking
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

// Joint position errors
float error1 = 0;
float error2 = 0;
float error3 = 0;
// Velocity errors
float vel_error1 = 0;
float vel_error2 = 0;
float vel_error3 = 0;

// Gains
float Kp1 = 55.0, Kd1 = 2.7, Ki1 = 500; //PID
float Kp2 = 70.0, Kd2 = 12, Ki2 = 500; //PID
float Kp3 = 70.0, Kd3 = 12, Ki3 = 500; //PID
float Kp2i = 1000, Kd2i = 100, Kp3i = 1000, Kd3i = 100; //Gains for Inverse Dynamics
float integral1 = 0,integral2 = 0,integral3 = 0; //PI Gains, not used
float threshold = 0.015; // used for integral deadband

//Motor Inertia Constants
float J1 = 0.0167,J2=0.03,J3 = 0.0128;

// Desired Velocity and Acceleration
float theta1_dot_desired = 0, theta2_dot_desired = 0, theta3_dot_desired = 0;
float theta1_ddot_desired = 0, theta2_ddot_desired = 0, theta3_ddot_desired = 0;

//Friction Compensation Parameters
float min_v1 = 0.1, min_v2 = 0.05, min_v3 = 0.05; // velocity thresholds
float u_fric1 = 0, u_fric2 = 0, u_fric3 = 0; // output torques from friction model
float Vp1 = 0.12, Vp2 = 0.34, Vp3 = 0.1; // positive direction friction coefficients
float Cp1 = 0.32, Cp2 = 0.30, Cp3 = 0.4; // positive direction friction coefficients
float Vn1 = 0.13, Vn2 = 0.36, Vn3 = 0.2; // negative direction friction coefficients
float Cn1 = 0.3, Cn2 = 0.55, Cn3 = 0.45; // negative direction friction coefficients
float slope = 3.6; //linear slope used near zero velocity (deadband zone)

// Dynamic Model Terms and Intermediate state
float sintheta2 = 0, costheta3 = 0, sintheta32 = 0, costheta32 = 0;
float p1 = 0.03, p2 = 0.0128, p3 = 0.0298, p4 = 0.0753, p5 = 0.0298;
float a_m2 = 0, a_m3 = 0;

// Trajectory Filter States
float step = 0, mystep = 0;
float qd = 0, dot = 0, ddot = 0;
float qd3 = 0, dot3 = 0, ddot3 = 0;

// Controll Mode Flag Variable
float  mode = 0;

void mains_code(void);

// Trajectory Filter
typedef struct steptraj_s {
    long double b[5];
    long double a[5];
    long double xk[5];
    long double yk[5];
    float qd_old;
    float qddot_old;
    int size;
} steptraj_t;

// Filter Coefficients
steptraj_t trajectory = {1.4781526816424225e-07L,5.9126107265696901e-07L,8.8689160898545357e-07L,5.9126107265696901e-07L,1.4781526816424225e-07L,
                         1.0000000000000000e+00L,-3.8431372549019605e+00L,5.5386389850057673e+00L,-3.5476249707880076e+00L,8.5212560572849194e-01L,
                         0,0,0,0,0,
                         0,0,0,0,0,
                         0,
                         0,
                         5};
steptraj_t trajectory2 = {1.4781526816424225e-07L,5.9126107265696901e-07L,8.8689160898545357e-07L,5.9126107265696901e-07L,1.4781526816424225e-07L,
                          1.0000000000000000e+00L,-3.8431372549019605e+00L,5.5386389850057673e+00L,-3.5476249707880076e+00L,8.5212560572849194e-01L,
                          0,0,0,0,0,
                          0,0,0,0,0,
                          0,
                          0,
                          5};

// Discrete-Time Filter Function
// this function must be called every 1ms.
void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
    int i = 0;

    traj->xk[0] = step;
    traj->yk[0] = traj->b[0]*traj->xk[0];
    for (i = 1;i<traj->size;i++) {
        traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
    }

    for (i = (traj->size-1);i>0;i--) {
        traj->xk[i] = traj->xk[i-1];
        traj->yk[i] = traj->yk[i-1];
    }

    *qd = traj->yk[0];
    *qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
    *qd_ddot = (*qd_dot - traj->qddot_old)*1000;

    traj->qd_old = *qd;
    traj->qddot_old = *qd_dot;

}


// to call this function create a variable that steps to the new positions you want to go to, pass this var to step
// pass a reference to your qd variable your qd_dot variable and your qd_double_dot variable
// for example
//  implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);

//
// Main
//
void main(void)
{
    mains_code();
}




// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    if ((mycount % 8000) < 4000) {
        mystep = 0.25;
        step = -.3;
    } else {
        mystep = 0.6;
        step = 0;
    }

    implement_discrete_tf(&trajectory, step, &qd3, &dot3, &ddot3);
    implement_discrete_tf(&trajectory2, mystep, &qd, &dot, &ddot);

    // Velocity Estimation
    Omega1_raw = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;

    Omega2_raw = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;

    Omega3_raw = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0;

    // Desired Postions, Velocities, Accelerations
    theta1_desired = qd;
    theta1_dot_desired = dot;
    theta1_ddot_desired = ddot;

    theta2_desired = qd;
    theta2_dot_desired = dot;
    theta2_ddot_desired = ddot;

    theta3_desired = qd3;
    theta3_dot_desired = dot3;
    theta3_ddot_desired = ddot3;
    
    // Position Errors
    error1 = theta1_desired - theta1motor;
    error2 = theta2_desired - theta2motor;
    error3 = theta3_desired - theta3motor;

    // Velocity Errors, added
    vel_error1 = theta1_dot_desired - Omega1;
    vel_error2 = theta2_dot_desired - Omega2;
    vel_error3 = theta3_dot_desired - Omega3;

    // Friction Compensation
    if (Omega1 > min_v1) {
        u_fric1 = Vp1*Omega1 + Cp1 ;
    } else if (Omega1 < -min_v1) {
        u_fric1 = Vn1*Omega1 - Cn1;
    } else {
        u_fric1 = slope*Omega1;
    }

    if (Omega2 > min_v2) {
        u_fric2 = Vp2*Omega2 + Cp2 ;
    } else if (Omega2 < -min_v2) {
        u_fric2 = Vn2*Omega2 - Cn2;
    } else {
        u_fric2 = slope*Omega2;
    }

    if (Omega3 > min_v3) {
        u_fric3 = Vp3*Omega3 + Cp3 ;
    } else if (Omega3 < -min_v3) {
        u_fric3 = Vn3*Omega3 - Cn3;
    } else {
        u_fric3 = slope*Omega3;
    }

    // Terms used the dynamics model
    sintheta2 = sin(theta2motor);
    costheta3 = cos(theta3motor);
    sintheta32 = sin(theta3motor - theta2motor);
    costheta32 = cos(theta3motor - theta2motor);
        
    // Control Laws
    if (mode == 0) {
        // PD + Feedforward Control
        *tau1 = J1 * theta1_ddot_desired
              + Kp1 * error1
              + Kd1 * vel_error1
              + u_fric1;

        *tau2 = J2 * theta2_ddot_desired
              + Kp2 * error2
              + Kd2 * vel_error2
              + u_fric2;

        *tau3 = J3 * theta3_ddot_desired
              + Kp3 * error3
              + Kd3 * vel_error3
              + u_fric3;

    } else {
        // InvIerse Dynamics Control
        a_m2 = theta2_ddot_desired + Kp2i * error2 + Kd2i * vel_error2;
        a_m3 = theta3_ddot_desired + Kp3i * error3 + Kd3i * vel_error3;

        *tau1 = J1 * theta1_ddot_desired
              + Kp1 * error1
              + Kd1 * vel_error1
              + u_fric1*0.6;

        *tau2 = -p3 * costheta32 * Omega3 * Omega3
              + a_m2 * p1
              - GRAV * p4 * sintheta2
              - a_m3 * p3 * sintheta32
              + u_fric2 * 0.6;

        *tau3 = p3 * costheta32 * Omega2 * Omega2
              + a_m3 * p2
              - GRAV * p5 * costheta3
              - a_m2 * p3 * sintheta32
              + u_fric3 * 0.6;
    }
    
    // Motor torque limitation(Max: 5 Min: -5)
    if (*tau1 > 5.0) *tau1 = 5.0;
    if (*tau1 < -5.0) *tau1 = -5.0;
    if (*tau2 > 5.0) *tau2 = 5.0;
    if (*tau2 < -5.0) *tau2 = -5.0;
    if (*tau3 > 5.0) *tau3 = 5.0;
    if (*tau3 < -5.0) *tau3 = -5.0;

    // Update old states for velocity filtering
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Theta3_old = theta3motor;
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

    // --- NOT Needed For Part 4---
    // x_end = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    // y_end = 0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    // z_end = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;
    // sigma1 = sqrt(x_end * x_end + y_end * y_end + (0.254 - z_end) * (0.254 - z_end));
    // cal_theta1 = atan2(y_end, x_end);
    // cal_theta2 = -(acos((2 * x_end * x_end + 2 * y_end * y_end) / (2 * sqrt(x_end * x_end + y_end * y_end) * sigma1)) + acos((x_end * x_end + y_end * y_end + (0.254 - z_end) * (0.254 - z_end)) / (2 * 0.254 * sigma1)));
    // cal_theta3 = acos((x_end * x_end - 2*0.254 * 0.254 + y_end * y_end + (0.254 - z_end) * (0.254 - z_end)) / (2 * 0.254 * 0.254));
    // cal_theta1motor = cal_theta1;
    // cal_theta2motor = cal_theta2 + PI/2;
    // cal_theta3motor = cal_theta3 + cal_theta2;
    
    // Simulink plotting
    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
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

