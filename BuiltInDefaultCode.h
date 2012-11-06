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
	Victor *m_shooterVictor1;
	Victor *m_shooterVictor2;
	Victor *m_elevatorVictor1;
	Victor *m_elevatorVictor2;
	
	// The xbox controller (this is probably what we will end up using)
	Joystick *m_xbox;
	Joystick *m_joystick;

	// Clock for autonomous mode
	UINT32 starttime;
	
	// This checks if the button was pressed in the last loop
	// Allows us to see when a button has been released rather than
	// pressed
	bool buttonLastPressed[22];
	bool buttonPressed[22];
	
	// Defines the variable used to point to the Compressor object
	Compressor *m_compressor;
	
	//MILKEN CODE
	// Pointers to the two solenoid objects for the drive gears and the two for the bridge mechanism
	Solenoid *m_driveGear1;
	Solenoid *m_driveGear2;
	Solenoid *m_bridgeMechanism1;
	Solenoid *m_bridgeMechanism2;
	
	// Tells us which gear is selected (1 or 2)
	int m_selectedGear;
	
	// The encoder
	Encoder *m_Encoder;
	
	// Multithreading stuff
	Task * m_vision;
	bool m_autoModeBegun;
	
	// Xbox controller sticks
	float m_LeftStickX;
	float m_LeftStickY;
	float m_RightStickX;
	float m_RightStickY;
	float m_Trig;
	float m_JoystickKnob;
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_visionPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;

	// The other speed that can be set with predetermined values
	float altShooterSpeed;
	
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
	
	// experimental driver station LCD
	DriverStationLCD* m_lcd;
	
	void victorPair(int n, bool on, bool rev);
	void victorPair(int n, float pow);
	void driveTank(float left, float right);

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
