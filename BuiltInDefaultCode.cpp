#include "WPILib.h"
#include "BuiltInDefaultCode.h"
#include <math.h>
#include <string.h>
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
////////////////////
#define JOYSTICK_DIFF 10
#define JOYSTICK_1	11
#define JOYSTICK_2	12
#define JOYSTICK_3	13
#define JOYSTICK_4	14
#define JOYSTICK_5	15
#define JOYSTICK_6	16
#define JOYSTICK_7	17
#define JOYSTICK_8	18
#define JOYSTICK_9	19
#define JOYSTICK_10	20
#define JOYSTICK_11	21
#define MAX_BUTTONS 21
#define JOYSTICK_SLIDE 3

// Where victors are located
#define VIC_LEFT1		9
#define VIC_LEFT2		10
#define VIC_RIGHT1		7
#define VIC_RIGHT2		8
#define VIC_INGEST1		3
#define VIC_INGEST2		4
#define VIC_SHOOTER1	1
#define VIC_SHOOTER2	2
#define VIC_ELEVATOR1	5 
#define VIC_ELEVATOR2	6

// These define the controls to the Compressor
// The Pressure Switch Channel is a GPIO Channel
// The Relay Switch is a relay switch channel
#define PRESSURE_SWITCH_CHANNEL 4
#define RELAY_SWITCH_CHANNEL 3
#define DEFAULT_GEAR 1

#define LUM_DEFAULT 224
#define SAT_DEFAULT 0
#define MIN_PARTICLE_SIZE 1000

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
	
//	m_frontLeftVictor = new Victor(9);
//	m_rearLeftVictor = new Victor(10);
//	m_frontRightVictor = new Victor(7);
//	m_rearRightVictor = new Victor(8);
	m_frontLeftVictor = new Victor(VIC_LEFT1);
	m_rearLeftVictor = new Victor(VIC_LEFT2);
	m_frontRightVictor = new Victor(VIC_RIGHT1);
	m_rearRightVictor = new Victor(VIC_RIGHT2);
	
	m_ingestionVictor1= new Victor(VIC_INGEST1);
	m_ingestionVictor2= new Victor(VIC_INGEST2);

	m_shooterVictor1 = new Victor(VIC_SHOOTER1);
	m_shooterVictor2 = new Victor(VIC_SHOOTER2);

	m_elevatorVictor1 = new Victor(VIC_ELEVATOR1);
	m_elevatorVictor2 = new Victor(VIC_ELEVATOR2);
	
	// Create a robot using standard right/left robot drive on PWMS 7, 8, 9, and 10
	// This uses victors which are declared above
	m_robotDrive = new RobotDrive(m_frontLeftVictor,
								  m_rearLeftVictor,
								  m_frontRightVictor,
								  m_rearRightVictor);
	
	// Acquire the Driver Station object
	m_ds = DriverStation::GetInstance();
	
	m_xbox = new Joystick(1);
	m_joystick = new Joystick(2);
	
	// Set up the compressor controls and the two solenoids
	m_compressor = new Compressor(PRESSURE_SWITCH_CHANNEL,RELAY_SWITCH_CHANNEL);
	m_driveGear1 = new Solenoid(1);
	m_driveGear2 = new Solenoid(2);
	m_bridgeMechanism1 = new Solenoid(3);
	m_bridgeMechanism2 = new Solenoid(4);
	m_selectedGear = DEFAULT_GEAR;
	m_selectedGear = DEFAULT_GEAR;
	
	// achannel and bchannel are set to 1 and 2, but those probably
	// aren't the right values
	m_Encoder = new Encoder(1,2,false,Encoder::k2X);
	
	if(DEFAULT_GEAR == 1)
	{
		m_driveGear1->Set(true);
		m_driveGear2->Set(false);
	} else {
		m_driveGear1->Set(false);
		m_driveGear2->Set(true);
	}
	
	m_bridgeMechanism1->Set(true);
	m_bridgeMechanism2->Set(false);
	
	// Set up the LCD (for getting output from the robot)
	m_lcd = DriverStationLCD::GetInstance();
	
	// set up the camera task
	m_vision = new Task("Vision",(FUNCPTR)StartTask);
	
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
	m_compressor->Start();

	printf("RobotInit() completed.\n");
}

void BuiltinDefaultCode::DisabledInit(void) {
	m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
	
	// Move the cursor down a few, since we'll move it back up in periodic.
	printf("\x1b[2B");
}

void BuiltinDefaultCode::AutonomousInit(void) {
	// Reset various counters
	m_autoPeriodicLoops = 0;
	selectedParticle = 0;
	bigParticles = 0;

	// Ideal numbers for saturation and luminosity
	// these represent the white vision targets
	sat = SAT_DEFAULT;
	lum = LUM_DEFAULT;

	if(!m_autoModeBegun)
	{
		m_autoModeBegun = true;
		m_vision->Start((int)this);
	}
}
	
void BuiltinDefaultCode::TeleopInit(void) {
	m_telePeriodicLoops = 0;			// Reset the loop counter for teleop mode
	if(!m_autoModeBegun)
	{
		m_autoModeBegun = true;
		m_vision->Start((int)this);
	}

	for(int i=1; i<=MAX_BUTTONS; i++)
	{
		buttonLastPressed[i] = false;
	}
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
			if(report.particleArea > MIN_PARTICLE_SIZE)
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
	// count number of times this routine has been called.
	m_autoPeriodicLoops++;

	if(m_autoPeriodicLoops < 400)
	{
		m_elevatorVictor1->Set(1);
		m_elevatorVictor2->Set(2);
	} else if(m_autoPeriodicLoops > 400 && m_autoPeriodicLoops < 600)
	{
		m_elevatorVictor1->Set(0);
		m_elevatorVictor2->Set(0);
		m_shooterVictor1->Set(0.5);
		m_shooterVictor1->Set(0.5);
	} else {
		m_elevatorVictor1->Set(0);
		m_elevatorVictor2->Set(0);
		m_shooterVictor1->Set(0);
		m_shooterVictor1->Set(0);
	}
	
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line6,"rev%d,AL:%d,VL:%d",CODE_REV,m_autoPeriodicLoops,m_visionPeriodicLoops);
	m_lcd->UpdateLCD();
}

void BuiltinDefaultCode::TeleopPeriodic(void) {
	// increment the number of teleop periodic loops completed
	m_telePeriodicLoops++;

	// Set the button values at the beginning of the loop
	// so that buttons are always consistent
	buttonPressed[XBOX_A]	= m_xbox->GetRawButton(XBOX_A);
	buttonPressed[XBOX_B]	= m_xbox->GetRawButton(XBOX_B);
	buttonPressed[XBOX_X]	= m_xbox->GetRawButton(XBOX_X);
	buttonPressed[XBOX_Y]	= m_xbox->GetRawButton(XBOX_Y);
	buttonPressed[XBOX_LB]	= m_xbox->GetRawButton(XBOX_LB);
	buttonPressed[XBOX_RB]	= m_xbox->GetRawButton(XBOX_RB);
	buttonPressed[XBOX_BACK]= m_xbox->GetRawButton(XBOX_BACK);
	buttonPressed[XBOX_START]=m_xbox->GetRawButton(XBOX_START);
	buttonPressed[XBOX_LJ]	= m_xbox->GetRawButton(XBOX_LJ);
	buttonPressed[XBOX_RJ]	= m_xbox->GetRawButton(XBOX_RJ);

	buttonPressed[JOYSTICK_1]	= m_joystick->GetRawButton(1);
	buttonPressed[JOYSTICK_2]	= m_joystick->GetRawButton(2);
	buttonPressed[JOYSTICK_3]	= m_joystick->GetRawButton(3);
	buttonPressed[JOYSTICK_4]	= m_joystick->GetRawButton(4);
	buttonPressed[JOYSTICK_5]	= m_joystick->GetRawButton(5);
	buttonPressed[JOYSTICK_6]	= m_joystick->GetRawButton(6);
	buttonPressed[JOYSTICK_7]	= m_joystick->GetRawButton(7);
	buttonPressed[JOYSTICK_8]	= m_joystick->GetRawButton(8);
	buttonPressed[JOYSTICK_9]	= m_joystick->GetRawButton(9);
	buttonPressed[JOYSTICK_10]	= m_joystick->GetRawButton(10);
	buttonPressed[JOYSTICK_11]	= m_joystick->GetRawButton(11);
	
	// Set the joystick variables
	m_LeftStickX	= m_xbox->GetRawAxis(XBOX_LSX);
	m_LeftStickY	= m_xbox->GetRawAxis(XBOX_LSY);
	m_RightStickX	= m_xbox->GetRawAxis(XBOX_RSX);
	m_RightStickY	= m_xbox->GetRawAxis(XBOX_RSY);
	m_Trig			= m_xbox->GetRawAxis(XBOX_TRIG);
	m_JoystickSlide	= m_joystick->GetRawAxis(JOYSTICK_SLIDE);

	// Drive gear, with solenoids
	// One of these is the slow gear, one is the fast gear
	// if right bumper was released, and solenoid 1 is on, turn on solenoid2
	if (!buttonPressed[XBOX_RB] && buttonLastPressed[XBOX_RB] && m_driveGear1->Get())
	{
		m_driveGear1->Set(false);
		m_driveGear2->Set(true);
		m_selectedGear = 2;
	}
	// same stuff here, with solenoid 1
	else if (!buttonPressed[XBOX_LB] && buttonLastPressed[XBOX_LB] && m_driveGear2->Get())
	{
		m_driveGear1->Set(true);
		m_driveGear2->Set(false);
		m_selectedGear = 1;
	}

	// Control the bridge mechanism with solenoids
	if((!buttonPressed[XBOX_LJ] && buttonLastPressed[XBOX_LJ]) ||
	   (!buttonPressed[JOYSTICK_11] && buttonLastPressed[JOYSTICK_11]))
	{
		if(m_bridgeMechanism1->Get())
		{
			m_bridgeMechanism1->Set(false);
			m_bridgeMechanism2->Set(true);
		} else {
			m_bridgeMechanism1->Set(true);
			m_bridgeMechanism2->Set(false);
		}
	}
	
	// Controls for vision target criteria
	// Not fully implemented
	if(!buttonPressed[JOYSTICK_8] && buttonLastPressed[JOYSTICK_8])
	{
		if(selectedParticle != 0)
		{
			selectedParticle--;
		}
	}
	else if(!buttonPressed[JOYSTICK_9] && buttonLastPressed[JOYSTICK_9])
	{
		if(selectedParticle < bigParticles)
		{
			selectedParticle++;
		}
	}

	if(buttonPressed[JOYSTICK_5])
	{
		if(lum < 219)
		{
			lum++;
		}
	}
	else if(buttonPressed[JOYSTICK_4])
	{
		if(lum > 0)
		{
			lum--;
		}
	}
	if(buttonPressed[JOYSTICK_3])
	{
		if(sat < 245)
		{
			sat++;
		}
	}
	else if(buttonPressed[JOYSTICK_2])
	{
		if(sat > 0)
		{
			sat--;
		}
	}
	if(buttonPressed[JOYSTICK_1])
	{
		// SHOOT THE BALL! TO VICTORY!
		// JoystickSlide normally goes from -1 to 1
		// so this makes sure we never get a negative value
		m_shooterVictor1->Set((m_JoystickSlide+1)/2);
		m_shooterVictor2->Set((m_JoystickSlide+1)/2);
	} else {
		m_shooterVictor1->Set(0);
		m_shooterVictor2->Set(0);
	}

	/*
	// Reset to the default values for the colors
	// if necessary
	if(buttonPressed[JOYSTICK_10])
	{
		lum = LUM_DEFAULT;
	}
	if(buttonPressed[JOYSTICK_11])
	{
		sat = SAT_DEFAULT;
	}
	*/
	float leftspeed;
	float rightspeed;
	
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

	// This adds for more variance in slower speed
	// makes sort of a hyperbolic graph when using the triggers for control
	if(m_Trig > 0.0001)
	{
		leftspeed   += pow(1.8,m_Trig)-0.8;
		rightspeed	+= pow(1.8,m_Trig)-0.8;
	} else if (m_Trig < -0.0001)
	{
		leftspeed	-= pow(1.8,fabs(m_Trig))-0.8;
		rightspeed	-= pow(1.8,fabs(m_Trig))-0.8;
	}
	
	if(leftspeed > 1)
	{
		leftspeed = 1;
	} else if(leftspeed < -1)
	{
		leftspeed = -1;
	}
	if(rightspeed > 1)
	{
		rightspeed = 1;
	} else if(rightspeed < -1)
	{
		rightspeed = -1;
	}

	if(buttonPressed[XBOX_A])
	{
		// Spin the robot to face the vision target
		// The normalized value gives us a value between
		// -1 and 1, so we can plug it in directly to the
		// rightspeed/leftspeed values
		rightspeed = selReport.center_mass_x_normalized;
		leftspeed = -rightspeed;
	}

	m_robotDrive->TankDrive(leftspeed,rightspeed);
	
	// Ingestion victors controlled by right joystick button OR the big joystick
	if ((!buttonPressed[XBOX_RJ] && buttonLastPressed[XBOX_RJ]) ||
		(!buttonPressed[JOYSTICK_10] && buttonPressed[JOYSTICK_10]))
	{
		if(!m_ingestionVictor1->Get())
		{
			m_ingestionVictor1->Set(1);
			m_ingestionVictor2->Set(1);
			m_elevatorVictor1->Set(1);
			m_elevatorVictor2->Set(1);
		} else {
			m_ingestionVictor1->Set(0);
			m_ingestionVictor2->Set(0);
			m_elevatorVictor1->Set(0);
			m_elevatorVictor2->Set(0);
		}
	}


	// set buttonLastPressed
	// This allows us to see if a button has been released rather than pressed
	for(int i=1; i<=MAX_BUTTONS; i++)
	{
		buttonLastPressed[i] = buttonPressed[i];
	}

	// Encoder stuff. I haven't tested this and I'm not entirely sure what it does
	//int encoderRaw = m_Encoder->GetRaw();
	
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line5,"%f",(m_JoystickSlide+1)/2);
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line6,"AL:%d,V:%d,G:%d",CODE_REV,m_telePeriodicLoops,m_visionPeriodicLoops,m_selectedGear);
	m_lcd->UpdateLCD();
	//END OF TELEOPERATED PERIODIC CODE (Not really)
}

START_ROBOT_CLASS(BuiltinDefaultCode);
