#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81
#define INCH2M      0.0254
#define MAXTORQUE   5

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -20.09*PI/180; // -0.37;
float offset_Enc3_rad = 14.72*PI/180; // 0.27;

// Your global varialbes.
long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// Basic variables
float theta[3] = {0, 0, 0};
float sintheta[3] = {0, 0, 0};
float costheta[3] = {0, 0, 0};

// For filtering veloicity
float theta_old[3] = {0, 0, 0};
float theta_raw[3] = {0, 0, 0};
float omega[3] = {0, 0, 0};
float omega_old[3][2] = {{0, 0}, {0, 0}, {0, 0}};
float omega_raw[3] = {0, 0, 0};

// For PID controller
float tau_input[3] = {0, 0, 0};

// Errors
float theta_err[3] = {0, 0, 0};
float omega_err[3] = {0, 0, 0};

// Moment of inertia
float J[3] = {.0167, .03, .0128};

// PD controller gains
//float Kp[3] = {66, 62, 88}; // motor1, motor2, motor3
//float Ki[3] = {0, 0, 0};
//float Kd[3] = {2.2, 2.2, 2.3};

// PID controller gains Part 1 & 2 & 3
float Kp[3] = {66, 75, 75}; // motor1, motor2, motor3
float Ki[3] = {0, 0, 0};
float Kd[3] = {4, 4, 4};
float theta_int[3] = {0, 0, 0};

// Feedforward controller
float q0 = 0;
float qf = 0;
float T = 0;
float t = 0;

// Desired thetamotor
float theta_des[3] = {0, 0, 0};
float omega_des[3] = {0, 0, 0};
float alpha_des[3] = {0, 0, 0};

// Friction compensation
float v_p[3] = {.145, .2, .235};
float v_n[3] = {.135, .2, .25};
float c_p[3] = {.45, .4759, .5};
float c_n[3] = {-.45, -.5031, -.5};
float thetamin[3] = {.1, .05, .05};
float u_fric[3] = {0, 0, 0};
float slope[3] = {3.6, 3.6, 3.6};

// Inverse dynamics: 0s are dummy values!
float p[5] = {.03, .0128, .0076, .0753, .0298};
float D[2][2] = {{0, 0}, {0, 0}};
float C[2][2] = {{0, 0}, {0, 0}};
float G[2] = {0, 0};
float a_theta[3] = {0, 0, 0};
float Kp_invdyn[3] = {0, 2000, 4000}; // motor1, motor2, motor3
float Kd_invdyn[3] = {0, 150, 250};
int invdyn_switch = 1; // 1 is  the inverse dynamics controller, 0 is the PD+FF controller

float q0_joint3 = 0;
float qf_joint3 = 0;

typedef struct steptraj_s {
    long double b[5];
    long double a[5];
    long double xk[5];
    long double yk[5];
    float qd_old;
    float qddot_old;
    int size;
} steptraj_t;

steptraj_t traj1 = {
    {9.6098034448281631e-09L, 3.8439213779312653e-08L, 5.7658820668968976e-08L, 3.8439213779312653e-08L, 9.6098034448281631e-09L},
    {1.0000000000000000e+00L, -3.9207920792079207e+00L, 5.7647289481423378e+00L, -3.7670505997761814e+00L, 9.2311388459861865e-01L},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    0,
    0,
    5
};

steptraj_t traj2 = {
    {9.6098034448281631e-09L, 3.8439213779312653e-08L, 5.7658820668968976e-08L, 3.8439213779312653e-08L, 9.6098034448281631e-09L},
    {1.0000000000000000e+00L, -3.9207920792079207e+00L, 5.7647289481423378e+00L, -3.7670505997761814e+00L, 9.2311388459861865e-01L},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    0,
    0,
    5
};

// this function must be called every 1ms.
void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
    int i = 0;
    traj->xk[0] = step;
    traj->yk[0] = traj->b[0] * traj->xk[0];

    for (i = 1; i < traj->size; i++) {
        traj->yk[0] = traj->yk[0] + traj->b[i] * traj->xk[i] - traj->a[i] * traj->yk[i];
    }

    for (i = (traj->size - 1); i > 0; i--) {
        traj->xk[i] = traj->xk[i - 1];
        traj->yk[i] = traj->yk[i - 1];
    }

    *qd = traj->yk[0];
    *qd_dot = (*qd - traj->qd_old) * 1000;  // 0.001 sample period
    *qd_ddot = (*qd_dot - traj->qddot_old) * 1000;
    traj->qd_old = *qd;
    traj->qddot_old = *qd_dot;
}

// to call this function create a variable that steps to the new positions you want to go to, pass this var to step
// pass a reference to your qd variable your qd_dot variable and your qd_double_dot variable
// for example
// implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);

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

    theta[0] = theta1motor;
    theta[1] = theta2motor;
    theta[2] = theta3motor;

    for (int i=0; i<3; i++) {
        sintheta[i] = sin(theta[i]);
        costheta[i] = cos(theta[i]);
    }

   // Filtered step trajectory
    if (mycount%8000 == 0) {
        qf = 0.25;
        qf_joint3 = 0.3;
    } else if (mycount%8000 == 4000) {
        qf = 0.85;
        qf_joint3 = -0.3;
    }

    implement_discrete_tf(&traj1 , qf, &theta_des[0], &omega_des[0], &alpha_des[0]);
    theta_des[1] = theta_des[0];
    omega_des[1] = omega_des[0];
    alpha_des[1] = alpha_des[0];
    implement_discrete_tf(&traj2 , qf_joint3, &theta_des[2], &omega_des[2], &alpha_des[2]);

    // Cubic trajectory
//    t = (mycount%1000)*.001;
//    if (mycount%2000 == 0) {
//        q0 = 0;
//        qf = .5;
//    } else if (mycount%2000 == 1000) {
//        q0 = .5;
//        qf = 0;
//    }
//    T = 1; // Period
//     for (int i=0; i<3; i++) {
//        theta_des[i] = q0 + 3*(qf - q0)*pow(t, 2)/pow(T, 2) - 2*(qf - q0)*pow(t, 3)/pow(T, 3);
//        omega_des[i] = 6*(qf - q0)*t/pow(T, 2) - 6*(qf - q0)*pow(t, 2)/pow(T, 3);
//        alpha_des[i] = 6*(qf - q0)/pow(T, 2) - 12*(qf - q0)*t/pow(T, 3);
//    }


    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    // Get motor velocity
    for (int i=0; i<3; i++) {
        omega_raw[i] = (theta[i] - theta_old[i])/.001;
        omega[i] = (omega_raw[i] + omega_old[i][0] + omega_old[i][1]) / 3.0;
        theta_err[i] = theta_des[i] - theta[i];
        omega_err[i] = omega_des[i] - omega[i];
        theta_old[i] = theta[i];
    }
    
    // Update omega_old
    for (int i=0; i<3; i++) {
        omega_old[i][1] = omega_old[i][0];
        omega_old[i][0] = omega_raw[i];
    }



    // Section 3
    D[0][0] = p[0];
    D[0][1] = -p[2]*sin(theta[2] - theta[1]);
    D[1][0] = -p[2]*sin(theta[2] - theta[1]);
    D[1][1] = p[1];

    C[0][0] = 0;
    C[0][1] = -p[2]*cos(theta[2] - theta[1])*omega[2];
    C[1][0] = p[2]*cos(theta[2] - theta[1])*omega[1];
    C[1][1] = 0;

    G[0] = -p[3]*GRAV*sintheta[1];
    G[1] = -p[4]*GRAV*costheta[2];

    for (int i=0; i<3; i++) {
        a_theta[i] = alpha_des[i] + Kp_invdyn[i]*theta_err[i] + Kd_invdyn[i]*omega_err[i];
    }
    a_theta[0] = 0; // Motor 1

    //  Calculate tau input (Feedforward)
    for (int i=0; i<3; i++) {
       tau_input[i] = J[i]*alpha_des[i] + Kp[i]*theta_err[i] + Ki[i]*theta_int[i] + Kd[i]*omega_err[i];
    }

    if (invdyn_switch == 1) { // Calculate tau input (Inverse dynamics)
    tau_input[1] = D[0][0]*a_theta[1] + D[0][1]*a_theta[2] + C[0][0]*omega[1] + C[0][1]*omega[2] + G[0];
    tau_input[2] = D[1][0]*a_theta[1] + D[1][1]*a_theta[2] + C[1][0]*omega[1] + C[1][1]*omega[2] + G[1];
    }

    // Friction compensation
    for (int i=0; i<3; i++) {
        if (omega[i] > thetamin[i]) {
            u_fric[i] = v_p[i]*omega[i] + c_p[i];
        } else if (omega[i] < -thetamin[i]) {
            u_fric[i] = v_n[i]*omega[i] + c_n[i];
        } else {
            u_fric[i] = slope[i]*omega[i];
        }
        tau_input[i] += u_fric[i];
    }

    // Motor limitation
    for (int i=0; i<3; i++) {
        if (tau_input[i] > MAXTORQUE) {
            tau_input[i] = MAXTORQUE;
        } else if (tau_input[i] < -MAXTORQUE) {
            tau_input[i] = -MAXTORQUE;
        }
    }
    *tau1 = tau_input[0];
    *tau2 = tau_input[1];
    *tau3 = tau_input[2];

    // save past states
        if ((mycount%50)==0) {

            theta1array[arrayindex] = theta1motor;

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

        printtheta1motor = theta[0];
        printtheta2motor = theta[1];
        printtheta3motor = theta[2];

        Simulink_PlotVar1 = theta_err[0];
        Simulink_PlotVar2 = theta_err[1];
        Simulink_PlotVar3 = theta_err[2];
        Simulink_PlotVar4 = 0;

        mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.2f\t%.2f\t%.2f\tEncoder\n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

