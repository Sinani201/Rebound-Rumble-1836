#ifndef BUILTINDEFAULTCODE_H
#define BUILTINDEFAULTCODE_H

class BuiltinDefaultCode : public IterativeRobot {
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;		// robot will use PWM 1-4 for drive motors
	
	Victor *m_frontLeftVictor;
	Victor *m_frontRightVictor;
	Victor *m_rearLeftVictor;
	Victor *m_rearRightVictor;
	Victor *m_ingestionVictor1;
	Victor *m_ingestionVictor2;
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	/*
	// Declare variables for the two joysticks being used
	Joystick *m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick *m_leftStick;			// joystick 2 (tank left stick)
	Joystick *m_armStick;			// joystick 3 (Arm Control stick)
	*/
	// The xbox controller (this is probably what we will end up using)
	Joystick *m_xbox;
	
	// This checks if the button was pressed in the last loop
	// Allows us to see when a button has been released rather than
	// pressed
	bool buttonLastPressed[10];
	
	//MILKEN CODE
	// Declare variables for the arm motors being used
	//Victor *arm_leftmotor;	
	//Victor *arm_rightmotor;
	// These define the two motors that control the minibot deployment system.
	//Victor *minibot_motor1;
	// Only one minibot motor
	// Victor *minibot_motor2;
	
	// Defines the variable used to point to the Compressor object used to open and close
	// the claw.
	Compressor *claw_compressor;
	
	//MILKEN CODE
	// Pointers to the two solenoid objects for the claw and the two for the minibot
	// delpoyment system
	Solenoid *m_Solenoid1;
	Solenoid *m_Solenoid2;
	Solenoid *m_Solenoid3;
	Solenoid *m_Solenoid4;
	
	int m_selectedGear;
	//Solenoid *m_MiniSolenoid1;
	//Solenoid *m_MiniSolenoid2;
	// Booleans (which are either true or false
	// These track if the Solenoids should on (true) or off (false)
	bool rightSolenoidOn;
	bool leftSolenoidOn;
	bool solenoid1On;
	bool solenoid2On;
	
	// The encoder
	Encoder *m_Encoder;
	
	// Multithreading stuff
	Task * m_vision;
	bool m_autoModeBegun;
	
	// variable used to hold the value of the arm movement speed.
	//float m_ArmJoyStickY;
	
	// Xbox controller sticks
	float m_LeftStickX;
	float m_LeftStickY;
	float m_RightStickX;
	float m_RightStickY;
	float m_Trig;
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_visionPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	char m_lastButton;
	
	// variable to count number of seconds elasped in autonomous mode 
	double m_auto_num_secs;
	
	// Camera stuff
	AxisCamera *camera;
	ParticleAnalysisReport report;
	ParticleAnalysisReport selReport;
	int numParticles;
	int bigParticles;
	int selectedParticle;
	int selParticleNum[80];
	int sat;
	int lum;
	
	// TEMP VICTORS
	//Victor *topleft;
	//Victor *rearleft;
	
	// All victors
	//Victor *one;
	//Victor *two;
	//Victor *three;
	Victor *four;
	//Victor *five;
	Victor *six;
	Victor *nine;
	Victor *ten;
	
	/*
	Victor *topright;
	Victor *rearright;
	*/
	//vector<ParticleAnalysisReport>* reports;
	
	// experimental driver station LCD
	DriverStationLCD* m_lcd;
	
	public:
	BuiltinDefaultCode(void);
	void RobotInit(void);
	void DisabledInit(void);
	void AutonomousInit(void);
	void TeleopInit(void);
	void RunAutoThread(void);
	void DisabledPeriodic(void);
	void AutonomousPeriodic(void);
	void TeleopPeriodic(void);
		
};
#endif
