#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.4138; // **Improved Accuracy** // Final Project Value: -0.4126; //Pre-set value: -0.37;
float offset_Enc3_rad = 0.2546; // **Improved Accuracy** // Final Project Value: 0.2466; // Pre-set Value: 0.27;

// Your global varialbes.

// World/base frame coordinates
float x = 0;
float y = 0;
float z = 0;

long mycount = 0;
long LEDcount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];
#pragma DATA_SECTION(theta3array, ".my_arrs")
float theta3array[100];

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

//************** Vel Joint-Space Coordinates Approximation **************
float Theta1_old = 0, Omega1_old1 = 0, Omega1_old2 = 0, Omega1 = 0;
float Theta2_old = 0, Omega2_old1 = 0, Omega2_old2 = 0, Omega2 = 0;
float Theta3_old = 0, Omega3_old1 = 0, Omega3_old2 = 0, Omega3 = 0;

//************** Pos & Vel Task-Space Coordinates Actual & Desired **************
float Dxd=0.0, Dyd=0.0, Dzd=0.0, Dx=0.0, Dy=0.0, Dz=0.0;// Derivative of world coordinates and desired world coordinates
float x_old=0.0, y_old=0.0, z_old=0.0;
float Dx_old1=0.0, Dy_old1=0.0, Dz_old1=0.0, Dx_old2=0.0, Dy_old2=0.0, Dz_old2=0.0;//Infinite velocity computation
float errX=0.0, errY=0.0, errZ=0.0, DerrX=0.0, DerrY=0.0, DerrZ=0.0;

//************ Task Space PD Controller Gains *******************
float Kpx = 0.75, Kpy = 0.9, Kpz = 1.3, Kdx = 0.0325, Kdy = 0.025, Kdz = 0.02;
float temp_Kpx = 0.75, temp_Kpy = 0.9, temp_Kpz = 1.05, temp_Kdx = 0.0325, temp_Kdy = 0.025, temp_Kdz = 0.03;

//************** Impedance Control Law **************
float PD[3] = {0.0,0.0,0.0};
float RTPD[3] = {0.0,0.0,0.0};
float JT[3][3] = { {0.0,0.0,0.0} , {0.0,0.0,0.0} , {0.0,0.0,0.0} }; // Initialize Jacobian Transpose Matrix
float cosq1=0.0, sinq1=0.0, cosq2=0.0, sinq2=0.0, cosq3=0.0, sinq3=0.0;// Jacobian Matrix Variables
float cosx=0.0, sinx=0.0, cosy=0.0, siny=0.0, cosz=0.0, sinz=0.0;// Rotation Matrix Variables
float R[3][3] = { {0.0,0.0,0.0} , {0.0,0.0,0.0} , {0.0,0.0,0.0} };
float RT[3][3] = { {0.0,0.0,0.0} , {0.0,0.0,0.0} , {0.0,0.0,0.0} };

//************** World Frame Rotations for Task-Space **************
float rotX=0.0, rotY=0.0, rotZ=0.0;
//float tempRotX = 0.0, tempRotY = 0.0, tempRotZ = 0.0;
float temp_RotZ = 0.0;

//************** Trajectroy Planning **************
unsigned int i = 0;
float xd = 0.0, yd = 0.0, zd = 0.0;
//float xa = 0.0, ya = 0.0, xb = 0.0, yb = 0.0, za = 0.0, zb = 0.0;
float t = 0.0;
float deltaX = 0.0, deltaY = 0.0, deltaZ = 0.0;
//float tStart = 0.0;
float tTotal = 0.0;
//float tHold = 0.0;
signed int dir = 1;
float sF = 1.0;

//************** Watch Expression Toqures **************
float mytau1 = 0.0;
float mytau2 = 0.0;
float mytau3 = 0.0;

//************** Motor Friction Compensation **************
float uFric1 = 0.0, uFric2 = 0.0, uFric3= 0.0;
float posVisc1 = 0.13, posVisc2 = 0.2, posVisc3 = 0.22;// Tuned friction coefficients
float negVisc1 = 0.13, negVisc2 = 0.23, negVisc3 = 0.22;
float negCoul1 = -0.25, negCoul2 = -0.543, negCoul3 = -0.498;
float posCoul1 = 0.25, posCoul2 = 0.475, posCoul3 = 0.5;
float minW1 = 0.1, minW2 = 0.05, minW3 = 0.05;
float slope = 3.6;

typedef struct traj{
	float ta;
	float tb;
	float xa;
	float ya;
	float za;
	float xb;
	float yb;
	float zb;
	float tempKpx;
	float tempKpy;
	float tempKpz;
	float tempKdx;
	float tempKdy;
	float tempKdz;
	float tempRotZ;
} point;
//=================       ta,   tb,    xa,    ya,    za,    xb,    yb,    zb,tempKpx,tempKpy,tempKpz,tempKdx,tempKdy,tempKdz,tempRotZ
point traj_pts[16] = {{  1.0,  2.0,  10.0,   0.0,  20.0,  1.18, 13.17,  7.35,   0.75,    0.9,   1.05, 0.0325,  0.025,   0.03,     0.0},
					  {  3.0,  4.0,  1.18, 13.17,  7.35,  1.18, 13.17,  4.46,    0.1,    0.1,   1.05,  0.005,  0.005,   0.03,     0.0},
					  {  5.0,  6.0,  1.18, 13.17,  4.46,  1.18, 13.17,  7.35,    0.1,    0.1,   1.05,  0.005,  0.005,   0.03,     0.0},
					  {  6.0,  7.0,  1.18, 13.17,  7.35,  8.00,  4.09, 14.00,   0.75,    0.9,   1.05, 0.0325,  0.025,   0.03,     0.0},
					  {  7.0,  8.0,  8.00,  4.09, 14.00, 14.96,  4.09,  7.80,   0.75,    0.9,   1.05, 0.0325,  0.025,   0.03,     0.0},
					  {  8.0,  8.5, 14.96,  4.09,  7.80, 16.07,  2.15,  7.80,   0.75,    0.1,   1.05, 0.0325,  0.005,   0.03, -1.1501},
					  {  8.5, 8.75, 16.07,  2.15,  7.80, 15.42,  1.42,  7.80,   0.75,    0.1,   1.05, 0.0325,  0.005,   0.03, -2.2983},
					  { 8.75, 9.25, 15.42,  1.42,  7.80, 13.08,  1.90,  7.80,   0.75,    0.1,   1.05, 0.0325,  0.005,   0.03,  2.2993},
					  { 9.25,  9.5, 13.08,  1.90,  7.80, 12.67,  1.52,  7.80,   0.75,    0.1,   1.05, 0.0325,  0.005,   0.03, -2.3942},
					  {  9.5, 9.75, 12.67,  1.52,  7.80, 12.58,  1.07,  7.80,   0.75,    0.1,   1.05, 0.0325,  0.005,   0.03, -1.7682},
					  { 9.75,10.25, 12.58,  1.07,  7.80, 15.17, -1.94,  7.80,   0.75,    0.1,   1.05, 0.0325,  0.005,   0.03, -0.8603},
					  {10.25, 11.0, 15.17, -1.94,  7.80, 15.17, -1.94, 13.87,   0.75,    0.9,   1.05, 0.0325,  0.025,   0.03,     0.0},
					  { 11.0, 11.5, 15.17, -1.94, 13.87, 14.55, -5.68, 13.87,   0.75,    0.9,   1.05, 0.0325,  0.025,   0.03,     0.0},
					  { 11.5, 12.5, 14.55, -5.68, 13.87, 14.55, -5.68, 12.75,   0.75,    0.9,   0.35, 0.0325,  0.025,   0.01,     0.0},
					  { 13.5, 14.0, 14.55, -5.68, 12.75, 14.55, -5.68, 13.87,   0.75,    0.9,   0.35, 0.0325,  0.025,   0.01,     0.0},
					  { 14.0, 15.0, 14.55, -5.68, 13.87,  10.0,   0.0,  20.0,   0.75,    0.9,   1.05, 0.0325,  0.025,   0.03,     0.0},
};

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int diffor)
{
	t = mycount*0.001;
	// Trajectory Planning,
	//---------------------------------------------------------------------------------------
	// Initial Position
	if(t <= (0.5/sF)){
		xd = 10.0;
		yd = 0.0;
		zd = 20.0;
	}

	// Update Step (Index)
	if((t >= (traj_pts[i].tb/sF)) && (i < 15)){ i++; }

	// Trajectory
	if((t >= (traj_pts[i].ta/sF)) && (t < (traj_pts[i].tb/sF))){
		tTotal = (traj_pts[i].tb/sF) - (traj_pts[i].ta/sF);
		deltaX = traj_pts[i].xb - traj_pts[i].xa;
		deltaY = traj_pts[i].yb - traj_pts[i].ya;
		deltaZ = traj_pts[i].zb - traj_pts[i].za;

		temp_Kpx = traj_pts[i].tempKpx;
		temp_Kpy = traj_pts[i].tempKpy;
		temp_Kpz = traj_pts[i].tempKpz;
		temp_Kdx = traj_pts[i].tempKdx;
		temp_Kdy = traj_pts[i].tempKdy;
		temp_Kdz = traj_pts[i].tempKdz;
		temp_RotZ = traj_pts[i].tempRotZ;

		xd = deltaX*(t - (traj_pts[i].ta/sF))/tTotal + traj_pts[i].xa;
		yd = deltaY*(t - (traj_pts[i].ta/sF))/tTotal + traj_pts[i].ya;
		zd = deltaZ*(t - (traj_pts[i].ta/sF))/tTotal + traj_pts[i].za;
	}

	// Reset
	if(t >= ((14.0/sF) + 3.0)){
		i = 0;
		t = 0.0;
	}

	cosq1 = cos(theta1motor);// Calculate sines and cosines once per lab call
	sinq1 = sin(theta1motor);
	cosq2 = cos(theta2motor);
	sinq2 = sin(theta2motor);
	cosq3 = cos(theta3motor);
	sinq3 = sin(theta3motor);

	rotZ = temp_RotZ;
	cosx = cos(rotX);
	sinx = sin(rotX);
	cosy = cos(rotY);
	siny = sin(rotY);
	cosz = cos(rotZ);
	sinz = sin(rotZ);

	//Forward Kinematics; inputs = motor angles FROM ENCODERS ; outputs = coordintes x, y, & z in the world/base frame
	//---------------------------------------------------------------------------------------
	x = 10*cosq1*(cosq3 + sinq2);
	y = 10*sinq1*(cosq3 + sinq2);
	z = 10*(1 + cosq2 - sinq3);

	JT[0][0] = -10*sinq1*(cosq3 + sinq2);
	JT[0][1] = 10*cosq1*(cosq3 + sinq2);
	JT[0][2] = 0;
	JT[1][0] = 10*cosq1*(cosq2 - sinq3);
	JT[1][1] = 10*sinq1*(cosq2 - sinq3);
	JT[1][2] = -10*(cosq3 + sinq2);
	JT[2][0] = -10*cosq1*sinq3;
	JT[2][1] = -10*sinq1*sinq3;
	JT[2][2] = -10*cosq3;

	R[0][0] = cosz*cosy - sinz*sinx*siny;
	R[0][1] = -sinz*cosx;
	R[0][2] = cosz*siny + sinz*sinx*cosy;
	R[1][0] = sinz*cosy + cosz*sinx*siny;
	R[1][1] = cosz*cosx;
	R[1][2] = sinz*siny - cosz*sinx*cosy;
	R[2][0] = -cosx*siny;
	R[2][1] = sinx;
	R[2][2] = cosx*cosy;

	RT[0][0] = R[0][0];
	RT[0][1] = R[1][0];
	RT[0][2] = R[2][0];
	RT[1][0] = R[0][1];
	RT[1][1] = R[1][1];
	RT[1][2] = R[2][1];
	RT[2][0] = R[0][2];
	RT[2][1] = R[1][2];
	RT[2][2] = R[2][2];

	// Infinite Average Velocity Implementation
	//---------------------------------------------------------------------------------------
	Omega1 = (theta1motor - Theta1_old)/0.001; // raw angular velocity
	Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0; // average with previous averages

	Omega2 = (theta2motor - Theta2_old)/0.001;
	Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;

	Omega3 = (theta3motor - Theta3_old)/0.001;
	Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;

	Dx = (x-x_old)/0.001;
	Dx = (Dx + Dx_old1 + Dx_old2)/3;

	Dy = (y-y_old)/0.001;
	Dy = (Dy + Dy_old1 + Dy_old2)/3;

	Dz = (z-z_old)/0.001;
	Dz = (Dz + Dz_old1 + Dz_old2)/3;

	// Friction coefficient calculations
	//---------------------------------------------------------------------------------------
	if(Omega1 > minW1){
		uFric1 = posVisc1*Omega1 + posCoul1;
	}else if(Omega1 < -minW1){
		uFric1 = negVisc1*Omega1 + negCoul1;
	}else{
		uFric1 = slope*Omega1;
	}
	if(Omega2 > minW2){
		uFric2 = posVisc2*Omega2 + posCoul2;
	}else if(Omega2 < -minW2){
		uFric2 = negVisc2*Omega2 + negCoul2;
	}else{
		uFric2 = slope*Omega2;
	}
	if(Omega3 > minW3){
		uFric3 = posVisc3*Omega3 + posCoul3;
	}else if(Omega3 < -minW3){
		uFric3 = negVisc3*Omega3 + negCoul3;
	}else{
		uFric3 = slope*Omega3;
	}

	errX = xd - x;
	errY = yd - y;
	errZ = zd - z;
	DerrX = Dxd - Dx;
	DerrY = Dyd - Dy;
	DerrZ = Dzd - Dz;

	// TORQUE CONTROLLERS
	//---------------------------------------------------------------------------------------
	Kpx = temp_Kpx;
	Kpy = temp_Kpy;
	Kpz = temp_Kpz;
	Kdx = temp_Kdx;
	Kdy = temp_Kdy;
	Kdz = temp_Kdz;

	PD[0] = Kpx*(RT[0][0]*errX + RT[0][1]*errY + RT[0][2]*errZ) + Kdx*(RT[0][0]*DerrX + RT[0][1]*DerrY + RT[0][2]*DerrZ);
	PD[1] = Kpy*(RT[1][0]*errX + RT[1][1]*errY + RT[1][2]*errZ) + Kdy*(RT[1][0]*DerrX + RT[1][1]*DerrY + RT[1][2]*DerrZ);
	PD[2] = Kpz*(RT[2][0]*errX + RT[2][1]*errY + RT[2][2]*errZ) + Kdz*(RT[2][0]*DerrX + RT[2][1]*DerrY + RT[2][2]*DerrZ);

	RTPD[0] = R[0][0]*PD[0] + R[0][1]*PD[1] + R[0][2]*PD[2];
	RTPD[1] = R[1][0]*PD[0] + R[1][1]*PD[1] + R[1][2]*PD[2];
	RTPD[2] = R[2][0]*PD[0] + R[2][1]*PD[1] + R[2][2]*PD[2];

	*tau1 = JT[0][0]*RTPD[0] + JT[0][1]*RTPD[1] + JT[0][2]*RTPD[2];
	*tau2 = JT[1][0]*RTPD[0] + JT[1][1]*RTPD[1] + JT[1][2]*RTPD[2];
	*tau3 = JT[2][0]*RTPD[0] + JT[2][1]*RTPD[1] + JT[2][2]*RTPD[2];

//	*tau1 = 0.0;
//	*tau2 = 0.0;
//	*tau3 = 0.0;

	*tau1 = *tau1 + 0.8*uFric1;// Feedforward + PD + Friction
	*tau2 = *tau2 + 0.8*uFric2;// Inverse Dynamics + Friction
	*tau3 = *tau3 + 0.8*uFric3;// Inverse Dynamics + Friction

	mytau1 = *tau1;// Watch expression torque values
	mytau2 = *tau2;
	mytau3 = *tau3;

	//Motor torque limitation(Max: 5 Min: -5)
	//---------------------------------------------------------------------------------------
	if( fabs(*tau1) > 5){
		if(*tau1 > 5){*tau1 = 5;}
		else{*tau1 = -5;}
	}
	if( fabs(*tau2) > 5){
		if(*tau2 > 5){*tau2 = 5;}
		else{*tau2 = -5;}
	}
	if( fabs(*tau3) > 5){
		if(*tau3 > 5){*tau3 = 5;}
		else{*tau3 = -5;}
	}

	// save past states
	if ((mycount%50)==0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;
		theta3array[arrayindex] = theta3motor;

		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}

	}

	if ((mycount%500)==0) {
		if (whattoprint > 0.5) {
			serial_printf(&SerialA, "I love robotics\n\r");
		} else {
			printtheta1motor = theta1motor;
			printtheta2motor = theta2motor;
			printtheta3motor = theta3motor;
			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		// GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}
	if (LEDcount==300) {
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
	}
	if (LEDcount==500){
		GpioDataRegs.GPBSET.bit.GPIO60 = 1;
	}
	if (LEDcount==600){
		GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
	}
	if (LEDcount==650){
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
		LEDcount = 0;
	}

	Simulink_PlotVar1 = xd;
	Simulink_PlotVar2 = yd;
	Simulink_PlotVar3 = zd;
	Simulink_PlotVar4 = tTotal;

	// ALL UPDATES
	Theta1_old = theta1motor;// approx. theta1 ang vel
	Omega1_old2 = Omega1_old1;
	Omega1_old1 = Omega1;
	Theta2_old = theta2motor;// approx. theta2 ang vel
	Omega2_old2 = Omega2_old1;
	Omega2_old1 = Omega2;
	Theta3_old = theta3motor;// approx. theta3 ang vel
	Omega3_old2 = Omega3_old1;
	Omega3_old1 = Omega3;
	x_old = x;
	y_old = y;
	z_old = z;
	Dx_old2 = Dx_old1;
	Dx_old1 = Dx;
	Dy_old2 = Dy_old1;
	Dy_old1 = Dy;
	Dz_old2 = Dz_old1;
	Dz_old1 = Dz;
	mycount++;// used for desired trajectories
	LEDcount++;
	// Re-Initialize temp Values
	temp_Kpx = 0.75;
	temp_Kpy = 0.9;
	temp_Kpz = 1.3;
	temp_Kdx = 0.0325;
	temp_Kdy = 0.025;
	temp_Kdz = 0.02;
	temp_RotZ = 0.0;
}

void printing(void){
	serial_printf(&SerialA, "x=%.2f, y=%.2f, z=%.2f, t1m=%.2f t2m=%.2f, t3m=%.2f \n\r",x,y,z,printtheta1motor,printtheta2motor, printtheta3motor);
}

