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
float Kp2i = 2000, Kd2i = 150, Kp3i = 5000, Kd3i = 150; //Gains for Inverse Dynamics
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
float  mode = 1;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = PI/4;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

float Kpx = 300, Kpy = 300, Kpz = 300;
float Kdx = 20, Kdy = 20, Kdz = 20;
float Kpxn = 300, Kpyn = 300, Kpzn = 300;
float Kdxn = 20, Kdyn = 20, Kdzn = 20;
float Fx = 0, Fy = 0, Fz = 0;
float Fxn = 0, Fyn = 0, Fzn = 0;
float x_end_old = 0, y_end_old = 0, z_end_old = 0;
float velx_raw = 0, vely_raw = 0, velz_raw = 0;
float velx_old1 = 0, vely_old1 = 0, velz_old1 = 0;
float velx_old2 = 0, vely_old2 = 0, velz_old2 = 0;
float velx = 0, vely = 0, velz = 0;

float xd = 0.25, yd = 0.25, zd = 0.35;
float xa = 0.25, ya = 0.25, za =0.50;
float xb = 0.25, yb = 0, zb =0.25;
float delta_x = 0, delta_y = 0, delta_z = 0;

float velxd = 0, velyd = 0, velzd = 0;
float Fzcmd = -10;
float Kt = 6;
float f_fric = 0.6;
float t = 0;
float t_total = 20;
float dis_total = 0;
float cycle_time = 0;
float t_mod = 0;

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


    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;
    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    t = mycount/1000.0;

    delta_x = xb-xa;
    delta_y = yb-ya;
    delta_z = zb-za;

    dis_total = sqrt(delta_x * delta_x+delta_y * delta_y+delta_z * delta_z);
    t_total  = dis_total/0.1;
    cycle_time = 2*t_total;
    t_mod = fmod(t, cycle_time);
    if (t_mod <=  t_total) {
        float ratio = t_mod / t_total;
        xd = delta_x * ratio + xa;
        yd = delta_y * ratio + ya;
        zd = delta_z * ratio + za;
    }

    else {
        float ratio = (t_mod -  t_total) /  t_total;
        xd = delta_x * (1 - ratio) + xa;
        yd = delta_y * (1 - ratio) + ya;
        zd = delta_z * (1 - ratio) + za;
    }
//    xd = delta_x * (t / t_total) + xa;
//    yd = delta_y * (t / t_total) + ya;
//    zd = delta_z * (t / t_total) + za;
    velxd = delta_x;
    velyd = delta_y;
    velzd = delta_z;
    // Forward Kinematic
    x_end = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y_end = 0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z_end = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;

    // Velocity Estimation
    Omega1_raw = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;

    Omega2_raw = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;

    Omega3_raw = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0;

    velx_raw = (x_end - x_end_old)/0.001;
    velx = (velx_raw + velx_old1 + velx_old2)/3.0;

    vely_raw = (y_end - y_end_old)/0.001;
    vely = (vely_raw + vely_old1 + vely_old2)/3.0;

    velz_raw = (z_end - z_end_old)/0.001;
    velz = (velz_raw + velz_old1 + velz_old2)/3.0;

    Fxn = Kpxn*(RT11*(xd-x_end)+RT12*(yd-y_end)+RT13*(zd-z_end))+Kdxn*(RT11*(velxd-velx)+RT12*(velyd-vely)+RT13*(velzd-velz));
    Fyn = Kpyn*(RT21*(xd-x_end)+RT22*(yd-y_end)+RT23*(zd-z_end))+Kdyn*(RT21*(velxd-velx)+RT22*(velyd-vely)+RT23*(velzd-velz));
    Fzn = Kpzn*(RT31*(xd-x_end)+RT32*(yd-y_end)+RT33*(zd-z_end))+Kdzn*(RT31*(velxd-velx)+RT32*(velyd-vely)+RT33*(velzd-velz));

    Fx = R11*Fxn + R12 * Fyn + R13 * Fzn;
    Fy = R21*Fxn + R22 * Fyn + R23 * Fzn;
    Fz = R31*Fxn + R32 * Fyn + R33 * Fzn;

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

        
    // Control Laws
    *tau1 = JT_11 * Fx + JT_12 * Fy + JT_13 * Fz + f_fric*u_fric1;
    *tau2 = JT_21 * Fx + JT_22 * Fy + JT_23 * Fz + f_fric*u_fric2;
    *tau3 = JT_31 * Fx + JT_32 * Fy + JT_33 * Fz + f_fric*u_fric3;


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

    x_end_old = x_end;
    velx_old2 = velx_old1;
    velx_old1 = velx;

    y_end_old = y_end;
    vely_old2 = vely_old1;
    vely_old1 = vely;

    z_end_old = z_end;
    velz_old2 = velz_old1;
    velz_old1 = velz;
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
    Simulink_PlotVar1 = qd;
    Simulink_PlotVar2 = qd3;
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

