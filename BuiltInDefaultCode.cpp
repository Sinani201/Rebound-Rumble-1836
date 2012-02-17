#include "WPILib.h"
#include "BuiltInDefaultCode.h"
#include <math.h>
#include <DriverStationLCD.h>

//MILKEN SPECIFIC definitions
// Code revision (if this didn't udpate on the driver
// station, the code didn't go through)
#define CODE_REV 1

// definitions of the xbox controllers buttons/triggers/sticks
#define XBOX_A		1
#define XBOX_B		2
#define XBOX_X		3
#define XBOX_Y		4
#define XBOX_LB		5
#define XBOX_RB		6
#define XBOX_BACK	7
#define XBOX_START	8
#define XBOX_LJ		9
#define XBOX_RJ		10
#define XBOX_LSX	1 // left stick x
#define XBOX_LSY	2 // left stick y
#define XBOX_TRIG	3 // left is positive, right is negative
#define XBOX_RSX	4 // right stick x
#define XBOX_RSY	5 // right stick y
#define XBOX_DPAD	6 // buggy

// These define which PWM controls are for the two arm motors
// It may not matter which is left and which is right
//#define RIGHT_ARM_MOTOR 6
//#define LEFT_ARM_MOTOR 5

// definitions of the motors for the MiniBot deployment system
// These define which PWM controls are used for each motor.
//#define MINIBOT_MOTOR1 7
// Now the deployment system uses only one motor.
// #define MINIBOT_MOTOR2 8
// Set the speeds of each deployment motor
// Single define means same speed in each direction
//#define MINIBOT_MOTOR1_SPEED 0.50
//#define MINIBOT_MOTOR2_SPEED 0.50

// definitions of the buttons for the MiniBot Motor Controls
//#define MINIBOT_MOTOR1_BUT1	11
//#define MINIBOT_MOTOR1_BUT2 10

// Now we only use one motor, but we do use buttons 7 and 6 for the solenoid
// #define MINIBOT_MOTOR2_BUT1 7
// #define MINIBOT_MOTOR2_BUT2 6
#define MINIBOT_SOLENOID1_BUT 7
#define MINIBOT_SOLENOID2_BUT 6

// define the minimum Arm Motor Speed
// Put in so the Arm doesn't jitter around
#define MIN_ARM_MOVE 0.0001 

// DRIVE CONTROL DEFINITIONS
// This defines the maximum turn angle that Joystick 1 can create. (Right stick.)
// This angle is acheived when the stick is moved completely to the left or right. (X-axis.)
#define MAX_TURN_DEGREES 30

// PNEUMATICS DEFINITIONS
// These define four solenoid connections.
// Not really RIGHT and LEFT.
// One opens and the other closes.
// Controlled (right now) by the top and trigger buttons on the arm joystick.
// The minibot delpoyment system uses the two solenoids now too!
#define RIGHT_SOLENOID 1
#define LEFT_SOLENOID 2
#define MINI_SOLENOID1 3
#define MINI_SOLENOID2 4

// These define the controls to the Compressor
// The Pressure Switch Channel is a GPIO Channel
// The Relay Switch is a relay switch channel
#define PRESSURE_SWITCH_CHANNEL 4
#define RELAY_SWITCH_CHANNEL 3

#define DEFAULT_GEAR 1

// Define the various constants that we will use during autonomous mode
#define AUTO_MODE_ARM_TIME 1  // # of seconds arm will move
#define AUTO_MODE_ARM_SPEED 0.10 // speed arm will move
#define AUTO_MODE_DRIVE_TIME 500 // # seconds robot will move
#define AUTO_MODE_DRIVE_SPEED 0.30 // speed (forward/reverse) that robot will move
#define AUTO_MODE_SPIN_TIME 500


/**
 * 
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "right" joystick.  Used for either "arcade drive" or "right" stick for tank drive
 *				In Mecanum drive this controls speed and turn angle.
 *   - USB 2 - The "left" joystick.  Used as the "left" stick for tank drive. Circles in Mecanum mode.
 * 	 - USB 3 - The "arm" joystick. Used to move the arm up and down.
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1/3 - Connected to "left" drive motor(s)
 *     - PWM 2/4 - Connected to "right" drive motor(s)
 * 		PWM 1 Front Left Drive Motor
 * 		PWM 3 Rear Left Drive Motor
 * 		PWM 2 Front Right Drive Motor
 * 		PWM 4 Rear Right Drive Motor
 */

// Start vision processing seperate task
int StartTask(BuiltinDefaultCode *bot)
{
	bot->RunAutoThread();
	return 0;
}

/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
BuiltinDefaultCode::BuiltinDefaultCode(void)	{
	printf("BuiltinDefaultCode Constructor Started\n");
	
	// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
	m_robotDrive = new RobotDrive(9, 10, 7, 8);
	
	// test for all Victors
	three = new Victor(3);
	four = new Victor(4);
	five = new Victor(5);
	six = new Victor(6);
	nine = new Victor(1);
	ten = new Victor(2);

	
	//topleft = new Victor(9);
	//topright = new Victor(10);
	//rearleft = new Victor(10);
	//rearright = new Victor(8);
	// Acquire the Driver Station object
	m_ds = DriverStation::GetInstance();
	m_priorPacketNumber = 0;
	m_dsPacketsReceivedInCurrentSecond = 0;
	
	// Define joysticks being used at USB ports #1, #2, and #3 on the Drivers Station
	//m_rightStick = new Joystick(1);
	//m_leftStick = new Joystick(2);
	//m_armStick = new Joystick(3);
	m_xbox = new Joystick(1);
	
	//MILKEN CODE
	//These are the controls for the motors that run the arm.
	//arm_leftmotor = new Victor(LEFT_ARM_MOTOR);
	//arm_rightmotor = new Victor(RIGHT_ARM_MOTOR);
	
	// Here we set up the two motors that control the minibot deployment system.
	//minibot_motor1 = new Victor(MINIBOT_MOTOR1);
	// NOW THERE IS NO SECOND MOTOR
	// minibot_motor2 = new Victor(MINIBOT_MOTOR2);
	
	// Make sure all motors are turned off. (Set to zero.) 
	//arm_leftmotor->Set(0);
	//arm_rightmotor->Set(0);
	//minibot_motor1->Set(0);
	// No second minibot Motor
	// minibot_motor2->Set(0);
	
	// Set up the compressor controls and the two solenoids
	claw_compressor = new Compressor(PRESSURE_SWITCH_CHANNEL,RELAY_SWITCH_CHANNEL);
	m_Solenoid1 = new Solenoid(1);
	m_Solenoid2 = new Solenoid(2);
	m_selectedGear = DEFAULT_GEAR;
	
	// achannel and bchannel are set to 1 and 2, but those probably
	// aren't the right values
	m_Encoder = new Encoder(1,2,false,Encoder::k2X);
	
	if(DEFAULT_GEAR == 1)
	{
		m_Solenoid1->Set(true);
		m_Solenoid2->Set(false);
	} else {
		m_Solenoid1->Set(false);
		m_Solenoid2->Set(true);
	}
	
	// Set up the two solenoids for the minibot deployment system
	//m_MiniSolenoid1 = new Solenoid(MINI_SOLENOID1);
	//m_MiniSolenoid2 = new Solenoid(MINI_SOLENOID2);
	
	// Set up the LCD (for getting output from the robot)
	m_lcd = DriverStationLCD::GetInstance();
	
	// Turn off all solenoids
	//m_Solenoid1->Set(0);
	//m_Solenoid2->Set(0);
	//m_MiniSolenoid1->Set(0);
	//m_MiniSolenoid2->Set(0);
	
	// set up the camera task
	m_vision = new Task("Vision",(FUNCPTR)StartTask);
	
	// Set the four Solenoid variables to their initial values
	// Set the solenoid that should be on initially here.
	// CHANGE THE CODE HERE BASED ON WHICH SOLENOID CLOSES THE CLAW
	// ONE OF THESE NEEDS TO BE SET TO TRUE
	// I AM NOT SURE WHICH ONE
	// We also need to figure out the initial conditions for the minibot solenoids
	//leftSolenoidOn = false;
	//rightSolenoidOn = true;
	//MiniSolenoid1On = true;
	//MiniSolenoid2On = false;
	
	// Initialize counters to record the number of loops completed in autonomous and teleop modes
	m_autoPeriodicLoops = 0;
	m_visionPeriodicLoops = 0;
	m_disabledPeriodicLoops = 0;
	m_telePeriodicLoops = 0;
	

	printf("BuiltinDefaultCode Constructor Completed\n");
}
	
/********************************** Init Routines *************************************/
void BuiltinDefaultCode::RobotInit(void) {
	// Actions which would be performed once (and only once) upon initialization of the
	// robot would be put here.

	// TURN ON THE COMPRESSOR
	claw_compressor->Start();

	// Set the solenoids to their initial conditions
	//m_Solenoid1->Set(rightSolenoidOn);
	//m_Solenoid2->Set(leftSolenoidOn);
	//m_MiniSolenoid1->Set(MiniSolenoid1On);
	//m_MiniSolenoid2->Set(MiniSolenoid2On);
	
	printf("RobotInit() completed.\n");
	//camera = &AxisCamera::GetInstance();
	//camera->WriteResolution(*camera.kResolution_320x240);
	//camera->WriteBrightness(50);
	//camera->WriteCompression(0);
	
	
}

void BuiltinDefaultCode::DisabledInit(void) {
	m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
	
	// Move the cursor down a few, since we'll move it back up in periodic.
	printf("\x1b[2B");
}

void BuiltinDefaultCode::AutonomousInit(void) {
	m_autoPeriodicLoops = 0;			// Reset the loop counter for autonomous mode
	selectedParticle = 0;
	bigParticles = 0;
	sat = 0;
	lum = 224;
	if(!m_autoModeBegun)
	{
		m_autoModeBegun = true;
		m_vision->Start((int)this);
	}
}
	
void BuiltinDefaultCode::TeleopInit(void) {
	m_telePeriodicLoops = 0;			// Reset the loop counter for teleop mode
	m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
	/*
	if(!m_autoModeBegun)
	{
		m_autoModeBegun = true;
		m_vision->Start((int)this);
	}
	*/
}

void BuiltinDefaultCode::RunAutoThread()
{
	while(1)
	{
		m_visionPeriodicLoops++;
		
		// VISION STUFF
		// Get the camera image
		ColorImage* camerain = camera->GetImage();
		
		// This threshold object doesn't work (not sure why)
		//Threshold* lightThreshold = new Threshold(0,9,0,5,250,255);
		
		// These threshold values track the white part of an image
		BinaryImage* binImage = camerain->ThresholdHSL(0,255,sat,sat+39,lum,lum+31);
		
		numParticles = binImage->GetNumberParticles();
		
		// Big particles are particles big enough to be considered a vision target
		// Right now the min. area is 500 but I might raise it if necessary
		bigParticles = 0;
				
		int i;
		
		// Get a report for each particle
		for(i=0; i<numParticles; i++)
		{
			binImage->GetParticleAnalysisReport(i,&report);
			if(report.particleArea > 1000)
			{
				selParticleNum[bigParticles] = i;
				bigParticles++;
			}
		}
		
		if(selectedParticle > bigParticles)
		{
			selectedParticle = bigParticles - 1;
		}
		
		// Later on, the code will find the right particle numers itself.
		// This version just takes particle 0 because in my tests
		// it has always been the largest
		if(selectedParticle >= 0)
		{
			binImage->GetParticleAnalysisReport(selParticleNum[selectedParticle],&selReport);
		}
		
		// Free memory
		delete binImage;
		delete camerain;
		
		// Values to get the x,y,width, and height of the 0th particle
		int x,y,w,h;
		
		// A = area. The API uses double but the numbers have always been whole for me.
		double a;
		
		// Set the values
		x = selReport.boundingRect.left;
		y = selReport.boundingRect.top;
		w = selReport.boundingRect.width;
		h = selReport.boundingRect.height;
		a = selReport.particleArea;
		
		m_lcd->PrintfLine(DriverStationLCD::kUser_Line1,"size:%f",a);
		m_lcd->PrintfLine(DriverStationLCD::kUser_Line2,"big:%d, all:%d",bigParticles,numParticles);
		m_lcd->PrintfLine(DriverStationLCD::kUser_Line3,"%d,%d,%d,%d",x,y,w,h);
		m_lcd->PrintfLine(DriverStationLCD::kUser_Line4,"%d/%d",selectedParticle+1,bigParticles);
		m_lcd->PrintfLine(DriverStationLCD::kUser_Line5,"Sat:%d Lum:%d",sat,lum);
		m_lcd->UpdateLCD();
				
		//binImage->Write("testimage4444.png");
		
		//BinaryImage* binImage = hoop->ThresholdHSL(0,9,0,5,250,255);
		
		//int numParticles = binImage->GetNumberParticles();
		//int found = 0;
		
		// We don't need these images anymore, so we can get rid of them.
		//delete camerain;
		/*
		int x=0,y=0,w=0,h=0;
		
		double largestParticle = 0;
		int largestParticleNum = -1;
		
		int bigParticles = 0;
		int numParticles;
		*/
		/*
		// Loop through each particle and see if it should be tracked
		//for(int i = 0; i<numParticles; i++)
		//{
		int i = 0;
		binImage->GetParticleAnalysisReport(i,&report);
		
		//if(largestParticle < report.particleToImagePercent);
		if(i == 0)
		{
			largestParticle = report.particleToImagePercent;
			largestParticleNum = i;
		}
		// Disregard small objects
		//if(report.particleToImagePercent > 4.5)
		//{
			// Get coordinates of the bounding rect
			x = report.boundingRect.left;
			y = report.boundingRect.top;
			w = report.boundingRect.width;
			h = report.boundingRect.height;
		//}
		//}
		*/
		//numParticles = binImage->GetNumberParticles();
		
		//double p1, p2, p3;
		/*
		for(int r=0; r<numParticles; r++)
		{
			binImage->GetParticleAnalysisReport(r,&report);
			if(report.particleArea > 7.0)
			{
				if(bigParticles==0)
				{
					p1=report.particleArea;
				} else if (bigParticles==1)
				{
					p2=report.particleArea;
				} else if (bigParticles==2)
				{
					p3=report.particleArea;
				}
				bigParticles++;
			}
		}
		*/
		/*
		binImage->GetParticleAnalysisReport(11,&report);
		// This holds the particle analysis reports
		largestParticle = report.particleArea;
		x = report.boundingRect.left;
		y = report.boundingRect.top;
		w = report.boundingRect.width;
		h = report.boundingRect.height;
		*/
		
		/*
		// Convert to an imaq image for more advanced operations
		Image* imaqBinImage = frcCreateImage(IMAQ_IMAGE_U8);
		imaqBinImage = binImage->GetImaqImage();
		//ThresholdHSL(0,10,0,10,250,255);
		*/
		
		/*
		Range hue,sat,lum;
		hue.minValue = 0;
		hue.maxValue = 10;
		sat.minValue = 0;
		sat.maxValue = 10;
		lum.minValue = 250;
		lum.maxValue = 255;
		*/
		//imaqBinImage = camerain->GetImaqImage();
		//frcWriteImage(imaqBinImage,"theimage2.png");

		//frcColorThreshold(imaqBinImage,imaqBinImage,IMAQ_HSL,&hue,&sat,&lum);
		//frcWriteImage(imaqBinImage,"theimage3.png");
		
		//imaqConvexHull(imaqBinImage,imaqBinImage,4);
		
		//int numParticles = 0;
		/*
		if(frcCountParticles(imaqBinImage,&numParticles) == 0)
		{
			numParticles = -1;
		}
		*/
		
		//frcParticleAnalysis(imaqBinImage, int particleNumber, ParticleAnalysisReport* par);
		//delete binImage;
		
		//m_lcd->PrintfLine(DriverStationLCD::kUser_Line2,"Particles: %d",numParticles);
		//m_lcd->PrintfLine(DriverStationLCD::kUser_Line3,"x,y,w,h: %d,%d,%d,%d",x,y,w,h);
		//m_lcd->PrintfLine(DriverStationLCD::kUser_Line4,"LPS: %f (%d)",largestParticle,&numParticles);
		//m_lcd->PrintfLine(DriverStationLCD::kUser_Line5,"BP:%d (%f,%f,%f)",bigParticles,p1,p2,p3);
	}
}

/********************************** Periodic Routines *************************************/

void BuiltinDefaultCode::DisabledPeriodic(void)  {
	static INT32 printSec = (INT32)GetClock() + 1;
	static const INT32 startSec = (INT32)GetClock();


	// increment the number of disabled periodic loops completed
	m_disabledPeriodicLoops++;
	
	// while disabled, printout the duration of current disabled mode in seconds
	if (GetClock() > printSec) {
		// Move the cursor back to the previous line and clear it.
		printf("\x1b[1A\x1b[2K");
		printf("Disabled seconds: %d\r\n", printSec - startSec);			
		printSec++;
	}
}

void BuiltinDefaultCode::AutonomousPeriodic(void) {
	/*
	// count number of times this routine has been called.
	// ++ causes a variable to be incremented (1 added to it.) 
	m_autoPeriodicLoops++;

	// figure out how many seconds have gone by in autonomous mode.
	m_auto_num_secs = (m_autoPeriodicLoops/GetLoopsPerSec());
			
	if (m_autoPeriodicLoops == 1) {
		// When on the first periodic loop in autonomous mode
		// start driving at AUTO_MODE_DRIVE_SPEED
		m_robotDrive->Drive(AUTO_MODE_DRIVE_SPEED, 0.0);
		
		// start at the arm moving
		arm_leftmotor->Set(AUTO_MODE_ARM_SPEED);
		arm_rightmotor->Set(AUTO_MODE_ARM_SPEED);
		}
	
		// If enough time has passed (based on AUTO_MODE_ARM_TIME)
		// stop the arm moving
		if (m_auto_num_secs >= AUTO_MODE_ARM_TIME) {
			arm_leftmotor->Set(0);
			arm_rightmotor->Set(0);
			}
		
		// If enough time has passed (based on AUTO_MODE_DRIVE_TIME)
		// stop the robot moving
		if (m_auto_num_secs >= AUTO_MODE_DRIVE_TIME) {
			m_robotDrive->Drive(0.0, 0.0);			// stop robot
			}
	 */

	//int numParticles = reports->size();
	
	
	
	if(m_xbox->GetRawButton(XBOX_LB))
	{
		if(selectedParticle != 0)
		{
			selectedParticle--;
		}
	}
	else if(m_xbox->GetRawButton(XBOX_RB))
	{
		if(selectedParticle < bigParticles)
		{
			selectedParticle++;
		}
	}
	
	if(m_xbox->GetRawButton(XBOX_Y))
	{
		if(lum < 219)
		{
			lum++;
		}
	}
	else if(m_xbox->GetRawButton(XBOX_A))
	{
		if(lum > 0)
		{
			lum--;
		}
	}
	if(m_xbox->GetRawButton(XBOX_START))
	{
		if(sat < 245)
		{
			sat++;
		}
	}
	else if(m_xbox->GetRawButton(XBOX_BACK))
	{
		if(sat > 0)
		{
			sat--;
		}
	}
	
	/*
	float spinspeed;
	if(m_xbox->GetRawButton(XBOX_X))
	{
		spinspeed = selReport.center_mass_x_normalized/3;
		m_robotDrive->MecanumDrive_Cartesian(0,0,spinspeed,0);
	} else {
		spinspeed = 0;
		m_robotDrive->MecanumDrive_Cartesian(0,0,0,0);
	}
	*/
	m_autoPeriodicLoops++;
	//m_lcd->PrintfLine(DriverStationLCD::kUser_Line1,"ALoops:%d", m_autoPeriodicLoops);
	//m_lcd->PrintfLine(DriverStationLCD::kUser_Line2,"VLoops:%d", m_visionPeriodicLoops);
	//m_lcd->PrintfLine(DriverStationLCD::kUser_Line3,"numParticles:%d", numParticles);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line6,"rev%d,AL:%d,VL:%d",CODE_REV,m_autoPeriodicLoops,m_visionPeriodicLoops);
	m_lcd->UpdateLCD();

	//double secondsPassed = m_autoPeriodicLoops / GetLoopsPerSec();
	// First move forward for a set amount of time
	
	// tesing printf
	//m_lcd->PrintfLine(DriverStationLCD::kUser_Linel, "Testing output", )
	
	/*
	if(m_autoPeriodicLoops < AUTO_MODE_DRIVE_TIME)
	{
		// This moves the robot forward at 0.1 speed (use tank drive instead of mecanum)
		m_robotDrive->MecanumDrive_Cartesian(0,AUTO_MODE_DRIVE_SPEED,0,0);
		//m_robotDrive->TankDrive(AUTO_MODE_DRIVE_SPEED,AUTO_MODE_DRIVE_SPEED);
	}
	else if(m_autoPeriodicLoops < (AUTO_MODE_SPIN_TIME + AUTO_MODE_DRIVE_TIME))
	{
		// This (should) spin the robot in place.
		m_robotDrive->MecanumDrive_Cartesian(0,0,AUTO_MODE_DRIVE_SPEED,0);
		//m_robotDrive->TankDrive(AUTO_MODE_DRIVE_SPEED,-(AUTO_MODE_DRIVE_SPEED));
	}
	else
	{
		m_robotDrive->MecanumDrive_Cartesian(0,0,0,0);
	}
	*/
}


void BuiltinDefaultCode::TeleopPeriodic(void) {
	// increment the number of teleop periodic loops completed
	m_telePeriodicLoops++;
			/*
	 * No longer needed since periodic loops are now synchronized with incoming packets.
	if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
	
		 
		 * Code placed in here will be called only when a new packet of information
		 * has been received by the Driver Station.  Any code which needs new information
		 * from the DS should go in here
		 
		 
		//m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
					
		// put Driver Station-dependent code here

		// Get the Y of the Arm joystick
		// This will be the speed and direction that the arm will move.
		//m_ArmJoyStickY = m_armStick->GetY();
		
		
		// Make sure arm doesn't jitter
		// The absolute value of the Y variable needs to be greater than the minimum.
		// If it is then pass it to the two motors.
		if (fabs(m_ArmJoyStickY)< MIN_ARM_MOVE) {
			arm_leftmotor->Set(0);
			arm_rightmotor->Set(0);
		} else {
			arm_leftmotor->Set(m_ArmJoyStickY);
			arm_rightmotor->Set(m_ArmJoyStickY);
		}
		
		//commented out drive code to test alternate joystick usage
		//USE ONLY ONE DRIVE METHOD
		// m_robotDrive->ArcadeDrive(m_rightStick);
		//m_robotDrive->TankDrive(m_leftStick,m_rightStick);
		
		//normal code to drive Mecanum wheels
		//See #defines at top for Maximum Angle
		//m_robotDrive->MecanumDrive_Cartesian(m_rightStick->GetX(),m_rightStick->GetY(),m_leftStick->GetY(),0);
		
		
		//Set Solenoids for Compressor Based on armStick buttons
		//Top Button Wins! (Since it is checked second.)
		if(m_armStick->GetTrigger()) 
			{
			rightSolenoidOn = true;
			leftSolenoidOn = false;
			}
		if(m_armStick->GetTop()) 
			{
			leftSolenoidOn = true;
			rightSolenoidOn = false;
			}
		
		
		//set the solenoids based on our variables			
		m_Solenoid1->Set(rightSolenoidOn);
		m_Solenoid2->Set(leftSolenoidOn);
		
		
		//Minibot Deployment System
		
		// Check Motor 1 Button 1 if on go at the predefined speed
		if(m_armStick->GetRawButton(MINIBOT_MOTOR1_BUT1))
			minibot_motor1->Set(MINIBOT_MOTOR1_SPEED);
		// Otherwise check Motor 1 Button 2 if on go at the predefined speed
		// in the other direction.
		else if (m_armStick->GetRawButton(MINIBOT_MOTOR1_BUT2))
			minibot_motor1->Set(0-MINIBOT_MOTOR1_SPEED);
		// Otherwise turn off motor 1
		else minibot_motor1->Set(0);
		
		// Second Minibot Motor currently not used.
		// Check Motor 2 Button 1 if on go at the predefined speed
		 if(m_armStick->GetRawButton(MINIBOT_MOTOR2_BUT1))
			minibot_motor2->Set(MINIBOT_MOTOR2_SPEED);
		// Otherwise check Motor 2 Button 2 if on go at the predefined speed
		// in the other direction.
		else if (m_armStick->GetRawButton(MINIBOT_MOTOR2_BUT2))
			minibot_motor2->Set(0-MINIBOT_MOTOR2_SPEED);
		// Otherwise turn off motor 2
		else minibot_motor2->Set(0);
		
		
		// Set the solenoids for the minibot delpoyment system based on the buttons
		// for that system
		// The second button wins.
		if(m_armStick->GetRawButton(MINIBOT_SOLENOID1_BUT))
		{
			MiniSolenoid1On = true;
			MiniSolenoid2On = false;
			
		}
		if (m_armStick->GetRawButton(MINIBOT_SOLENOID2_BUT))
		{
			MiniSolenoid1On = false;
			MiniSolenoid2On = true;
			
		}
		
		// Set the solenoids to the current value of their control variables.
		*/
		
		if(m_xbox->GetRawButton(XBOX_A))
		{
			m_lastButton='a';
		}
		if(m_xbox->GetRawButton(XBOX_B))
		{
			m_lastButton = 'b';
		}
		//if(m_lastButton == 'b');
		//{
		
		if (m_pressb && !m_xbox->GetRawButton(XBOX_B))
		{
			if(solenoid1On)
			{
				solenoid1On = false;
				solenoid2On = true;
				m_selectedGear = 2;
			} else {
				solenoid1On = true;
				solenoid2On = false;
				m_selectedGear = 1;
			}
			m_lastButton='b';
		} 
		//}
		if (m_xbox->GetRawButton(XBOX_X))
		{
			m_lastButton='x';
		} else if (m_xbox->GetRawButton(XBOX_Y))
		{
			m_lastButton='y';
		} else if (m_xbox->GetRawButton(XBOX_LB))
		{
			m_lastButton='l';
		} else if (m_xbox->GetRawButton(XBOX_RB))
		{
			m_lastButton='r';
		} else if (m_xbox->GetRawButton(XBOX_BACK))
		{
			m_lastButton='<';
		} else if (m_xbox->GetRawButton(XBOX_START))
		{
			m_lastButton='>';
		} else if (m_xbox->GetRawButton(XBOX_LJ))
		{
			m_lastButton='L';
		} else if (m_xbox->GetRawButton(XBOX_RJ))
		{
			m_lastButton='R';
		}
	
	m_LeftStickX = m_xbox->GetRawAxis(XBOX_LSX);
	m_LeftStickY = m_xbox->GetRawAxis(XBOX_LSY);
	m_RightStickX= m_xbox->GetRawAxis(XBOX_RSX);
	m_RightStickY= m_xbox->GetRawAxis(XBOX_RSY);
	m_ArmStickY  = m_xbox->GetRawAxis(XBOX_TRIG);
	m_Solenoid1->Set(solenoid1On);
	m_Solenoid2->Set(solenoid2On);
	
	/*
	// Make sure the arm doesn't jitter
	if (fabs(m_ArmStickY)< MIN_ARM_MOVE) {
		arm_leftmotor->Set(0);
		arm_rightmotor->Set(0);
	} else {
		arm_leftmotor->Set(m_ArmStickY);
		arm_rightmotor->Set(m_ArmStickY);
	}
	*/
	
	//m_robotDrive->MecanumDrive_Cartesian(m_RightStickX,m_RightStickY,m_LeftStickY,0);
	
	/*
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line1,"TLoops:%d", m_telePeriodicLoops);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line2,"XboxButton:%c",button);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line3,"LeftStick:%fx%f",m_LeftStickX,m_LeftStickY);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4,"RightStick:%fx%f",m_RightStickX,m_RightStickY);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line5,"Trig:%f",m_ArmStickY);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line6,"rev%d",CODE_REV);
	m_lcd->UpdateLCD();
	*/
	
	if(m_xbox->GetRawButton(XBOX_LB))
	{
		if(selectedParticle != 0)
		{
			selectedParticle--;
		}
	}
	else if(m_xbox->GetRawButton(XBOX_RB))
	{
		if(selectedParticle < bigParticles)
		{
			selectedParticle++;
		}
	}
	
	if(m_xbox->GetRawButton(XBOX_Y))
	{
		if(lum < 219)
		{
			lum++;
		}
	}
	else if(m_xbox->GetRawButton(XBOX_A))
	{
		if(lum > 0)
		{
			lum--;
		}
	}
	if(m_xbox->GetRawButton(XBOX_START))
	{
		if(sat < 245)
		{
			sat++;
		}
	}
	else if(m_xbox->GetRawButton(XBOX_BACK))
	{
		if(sat > 0)
		{
			sat--;
		}
	}
	float spinspeed;
	float leftspeed;
	float rightspeed;
	
	if(m_xbox->GetRawButton(XBOX_X))
	{
		spinspeed = selReport.center_mass_x_normalized/3;
		//m_robotDrive->MecanumDrive_Cartesian(0,0,spinspeed,0);
		//m_robotDrive->TankDrive(spinspeed,-spinspeed);
		m_robotDrive->TankDrive(1,1);
	} else {
		spinspeed = 0;
		// prevent jitter
		if(fabs(m_LeftStickY) >= 0.001)
		{
			leftspeed = m_LeftStickY;
		} else {
			leftspeed = 0;
		}
		if(fabs(m_RightStickY) >= 0.001)
		{
			rightspeed = m_RightStickY;
		} else {
			rightspeed = 0;
		}
		m_robotDrive->TankDrive(leftspeed,rightspeed);
	}
	
	if(m_xbox->GetRawButton(XBOX_B))
	{
		m_pressb = true;
	} else {
		m_pressb = false;
	}
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4,"left: %f",m_LeftStickY);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line5,"right:%f",m_RightStickY);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line6,"rev%d,AL:%d,VL:%d,G:%d",CODE_REV,m_telePeriodicLoops,m_visionPeriodicLoops,m_selectedGear);
	m_lcd->UpdateLCD();
	//END OF TELEOPERATED PERIODIC CODE
}

START_ROBOT_CLASS(BuiltinDefaultCode);
