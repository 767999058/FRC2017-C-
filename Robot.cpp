#include "WPILib.h"
#include<iostream>
#include<math.h>
#include<spi.h>
#include <chrono>
#include<RiptideRecorder/RiptideRecorder.h>
using namespace std;
using namespace std::chrono;
using namespace frc;

//#include"pixy.h"

/*
 Bytes    16-bit word    Description
 ----------------------------------------------------------------
 0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
 2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
 4, 5     y              signature number
 6, 7     y              x center of object
 8, 9     y              y center of object
 10, 11   y              width of object
 12, 13   y              height of object
 */

//Default address of Pixy Camera. You can change the address of the Pixy in Pixymon under setting-> Interface
#define PIXY_I2C_DEFAULT_ADDR           0x54

// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L //x: 0~319 pixels, y:0~199 pixels. (0,0) starts at bottom left
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L

// RC-servo values - not needed unless you want to use servo to face the goal instead of moving the whole robot
#define PIXY_RCS_MIN_POS            0L
#define PIXY_RCS_MAX_POS            1000L
#define PIXY_RCS_CENTER_POS         ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
unsigned int objectSize;
unsigned long pixyTime;

enum BlockType {
	NORMAL_BLOCK, //normal color recognition
	CC_BLOCK   //color-code(chnage in angle) recognition
};

struct Block {
	// print block structure!
	void print() {
		int i, j;
		char buf[128], sig[6], d;
		bool flag;
		if (signature > PIXY_MAX_SIGNATURE) // color code! (CC)
		{
			// convert signature number to an octal string
			for (i = 12, j = 0, flag = false; i >= 0; i -= 3) //assigns value to signature, x, y, width, height, and anlge
					{
				d = (signature >> i) & 0x07;
				if (d > 0 && !flag)
					flag = true;
				if (flag)
					sig[j++] = d + '0';
			}
			sig[j] = '\0';
			printf(
					"CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d\n",
					sig, signature, x, y, width, height, angle);
		} else
			// regular block.  Note, angle is always zero, so no need to print
			printf("sig: %d x: %d y: %d width: %d height: %d\n", signature, x,
					y, width, height); //prints out data to console instead of smartDashboard -> check on the side of the driver station, check +print and click view console
		//Serial.print(buf);
	}
	uint16_t signature; //Identification number for your object - you could set it in the pixymon
	uint16_t x; //0 - 320
	uint16_t y; //0 - 200
	uint16_t width;
	uint16_t height;
	uint16_t angle;

};

class Robot: public frc::IterativeRobot {
public:
	Robot() :
			autoRecorder() {

	}

	/**
	 * Mecanum drive is used with the gyro angle as an input.
	 */
	bool getStart() //checks whether if it is start of the normal frame, CC frame, or the data is out of sync
	{
		uint16_t w, lastw;

		lastw = 0xffff;

		while (true) {
			w = getWord(); //This it the function right underneath
			if (w == 0 && lastw == 0) {
				//delayMicroseconds(10);
				return false;
			} else if (w == PIXY_START_WORD && lastw == PIXY_START_WORD) {
				blockType = NORMAL_BLOCK;
				return true;
			} else if (w == PIXY_START_WORD_CC && lastw == PIXY_START_WORD) {
				blockType = CC_BLOCK;
				return true;
			} else if (w == PIXY_START_WORDX) //when byte recieved was 0x55aa instead of otherway around, the code syncs the byte
			{
				printf("Pixy: reorder");
				getByte(); // resync
			}
			lastw = w;
		}
	}

	uint16_t getWord() //Getting two Bytes from Pixy (The full information)
	{
		unsigned char buffer[2] = { 0, 0 };

		i2c->ReadOnly(2, buffer);
		return (buffer[1] << 8) | buffer[0]; //shift buffer[1] by 8 bits and add( | is bitwise or) buffer[0] to it
	}

	uint8_t getByte() //gets a byte
	{
		unsigned char buffer[1] = { 0 };

		i2c->ReadOnly(1, buffer);
		return buffer[0];
	}

	uint16_t getBlocks(uint16_t maxBlocks) {
		blocks[0] = {0}; //resets the array - clears out data from previous reading
		uint8_t i;
		uint16_t w, checksum, sum;
		Block *block;

		if (!skipStart)//when computer has not seen 0xaa55 (starting frame)
		{
			if (getStart()==false)
			return 0;
		}
		else
		skipStart = false;

		for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
		{
			checksum = getWord();
			if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame - checking for 0xaa55
			{
				skipStart = true; //starts this function
				blockType = NORMAL_BLOCK;
				//Serial.println("skip");
				return blockCount;
			}
			else if (checksum==PIXY_START_WORD_CC) //we've reacehd the beginning of the next frame - checking for 0xaa56
			{
				skipStart = true;
				blockType = CC_BLOCK;
				return blockCount;
			}
			else if (checksum==0)
			return blockCount;

			//if (blockCount>blockArraySize)
			//resize();

			block = blocks + blockCount;

			for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
			{
				if (blockType==NORMAL_BLOCK && i>=5) // skip --if not an CC block, no need to consider angle
				{
					block->angle = 0;
					break;
				}
				w = getWord();
				sum += w; //sum = w + sum
				*((uint16_t *)block + i) = w;//converts block to interger value
			}
			if (checksum==sum)
			blockCount++;
			else
			printf("Pixy: cs error");

			w = getWord(); //when this is start of the frame
			if (w==PIXY_START_WORD)
			blockType = NORMAL_BLOCK;
			else if (w==PIXY_START_WORD_CC)
			blockType = CC_BLOCK;
			else
			return blockCount;
		}
	}

	void centerServo()
	{
		if (panServoPos > panServoCenter) // center pan servo
		{
			panServoPos -= 2.25;
		}
		if (panServoPos < panServoCenter)  // center pan servo
		{
			panServoPos += 2.25;
		}
		if (tiltServoPos > tiltServoCenter) // center tilt servo
		{
			tiltServoPos -= 2.25;
		}
		if (tiltServoPos < tiltServoCenter) // center tilt servo
		{
			tiltServoPos += 2.25;
		}
	}

//	void servoTrack()
//	{
////	    	static int i = 0;
////	    	  int j;
//
////	    	  char buf[32];
//		int panError, tiltError;
//		panError = X_CENTER-blocks[0].x;
//		tiltError = blocks[0].y-Y_CENTER;
//
//		panLoop.update(panError);
//		tiltLoop.update(tiltError);
//
////	    	      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
//		panServo.Set(panLoop.m_pos);
//		tiltServo.Set(tiltLoop.m_pos);
//
//	}

	void loopTrackWithPanTiltServo()
	{

		double sensitivity = 1.5;

		double AngleSensitivity = 1.5;
		Forward =false;
		Backward = false;
		Left = false;
		TurnLeft = false;
		TurnRight = false;
		Right = false;
		Run = false;
		isVisionOn =false;
		isTwo =false;
//
//		if (blocks[0].x&&blocks[1].x)
//		{
//			isTwo =true;
//		}
//		else
//		{
//			isTwo = false;
//		}

		if (blocks->signature ==1 )
		{

			isVisionOn = true;
			objectSize = blocks->x * blocks->y;
			cout<<"Current Object Size: "<<objectSize<<endl;

			if (blocks[0].x > 150 && panServoPos > panServoLeft)
			{
				panServoPos -= 2.25*AngleSensitivity; //Object in the Right, needs left
			}
			if (blocks[0].x < 160 && panServoPos < panServoRight)
			{
				panServoPos += 2.25*AngleSensitivity;
			}
			if (blocks[0].y > 90 && tiltServoPos > tiltServoUp)
			{
				tiltServoPos += 2.25*AngleSensitivity;
			}

			if (blocks[0].y < 110 && tiltServoPos < tiltServoDown)
			{
				tiltServoPos -= 2.25*AngleSensitivity;
			}

			if (currentDistance >500)
			{
//				pidController.Enable();

				if(panServoPos < panServoCenter+32*sensitivity && panServoPos > panServoCenter-32*sensitivity)
				{
//					goForward();
//					pidControllerVision->Enable();

				}

				if (panServoPos >= panServoCenter+32*sensitivity)
				{
//	    	            left();
//					shiftLeft();

//					pidControllerVision->Enable();

				}
//				if (panServoPos >= panServoCenter+32*turnSensitivity)
//				{
//					//	    	            left();
//					pidController.Disable();
//					turnLeft();
//					pidControllerVision->Enable();
//				}
				if (panServoPos <= panServoCenter-32*sensitivity)
				{
//	    	            right();
//					shiftRight();
//					pidControllerVision->Enable();
				}
//				if (panServoPos <= panServoCenter-32*turnSensitivity)
//				{
////	    	            right();
//
//					turnRight();
////					pidControllerVision->Enable();
//				}

			}

			if (currentDistance<=500)    // object close to robot

//			{
//				stopRobot();
//			}

			{
//				pidController.Enable();
//				if (currentDistance < 400)
//				{
//					pidControllerUltra->SetSetpoint(350);
//					pidControllerUltra->SetAbsoluteTolerance(10);
//					pidControllerUltra->Enable();
//					cout<<"UltraPID Executed. "<<endl;
//				}
				if(panServoPos < panServoCenter+32*sensitivity && panServoPos > panServoCenter-32*sensitivity)
				{
//	    	            backward();
//					goBackward();
//					pidControllerUltra->Enable();
//					stopRobot();
//					pidControllerVision->Enable();

				}
				if (panServoPos >= panServoCenter+32*sensitivity)
				{
//	    	            left();
//					shiftLeft();
//					pidControllerVision->Enable();

				}
				if (panServoPos <= panServoCenter-32*sensitivity)
				{
//	    	            right();
//					shiftRight();
//					pidControllerVision->Enable();

				}

			}
		}
		else
		{
//			pidControllerVision->Disable();
			cout<<"Nothing Found!"<<endl;
			centerServo();
			cout<<"Ran CenterServo: "<<endl;
//			stopRobot();
//			turnRight();
		}

		panServo.SetAngle(panServoPos);
		tiltServo.SetAngle(tiltServoPos);

		cout<<"Pan: "<<panServoPos<<endl;
		cout<<"Tilt: "<<tiltServoPos<<endl;

	}

	void followBlock()
	{
		Forward = false;
		Backward = false;
		Left = false;
		Right = false;
		Run = false;
		bool FoundTwo = false;
		double target = (blocks[0].x + blocks[1].x)/2;
//		if(blocks[0].x && blocks[1].x)
//		{
//			FoundTwo = true;
//		}
//	    	pidController.SetSetpoint(0);
//	        	pidController.Enable();

//	        	servoTrack();
		if (blocks->signature == 2 )

		{
//			pidController.Enable();
//			pidControllerVision->Enable();

			cout<<"Object Found"<<endl;

			if (target >= 155 && target <=165)
			{

				cout<<"Middle"<<endl;
				cout<<"Stop"<<endl;

				if (currentDistance >500)
				{
					cout<<"Forward"<<endl;
					Forward = true;
					cout<<"Executed"<<endl;
					goForward();
					return;
				}
				else
				{
					stopRobot();
				}

			}

			if(target > 165)
			{
				if (target>190)
				{
					turnRight();
					return;
				}
				shiftRight();
				pidController.Enable();

			}

			if(target <155)
			{
				if(blocks->x<130)
				{
					turnLeft();
					return;

				}
				shiftLeft();
				pidController.Enable();

			}

		}
		else
		{
			cout<<"Nothing Found!"<<endl;
			stopRobot();
		}
	}

	void goForward ()
	{
		Forward = true;
		Run = true;
		myRobot.MecanumDrive_Cartesian(0,-0.6,0,gyro.GetAngle());
//		Wait(0.5);
		cout<<"Forward"<<endl;
//	    	Wait(.5);
	}

	void goBackward()
	{
		Backward = true;
		Run = true;
		myRobot.MecanumDrive_Cartesian(0, 0.6, 0, gyro.GetAngle());
//		Wait(0.5);
		cout<<"Backward"<<endl;
	}

	void shiftLeft()
	{
		cout<<"Shift Left"<<endl;
		Left = true;
		Run = true;

		myRobot.MecanumDrive_Cartesian(-0.6,0,0);
//		Wait(0.5);
//	    	  Wait(.5);
		cout<<"Left"<<endl;

	}

	void shiftRight()
	{
		cout<<"Shift Right"<<endl;
		Right = true;
		Run = true;

		myRobot.MecanumDrive_Cartesian(0.6,0,0);
//		Wait(0.5);
//	    	 Wait(.5);
		cout<<"Right"<<endl;

	}
	void turnRight()
	{

		Run = true;
		TurnRight = true;
		cout<<"Turn Right"<<endl;
//		pidController.SetSetpoint(gyro.GetAngle());
		myRobot.MecanumDrive_Cartesian(0,0,0.45,gyro.GetAngle());

//		myRobot.MecanumDrive_Cartesian(0,0,0);
	}

	void turnLeft()
	{
		Run = true;
		TurnLeft = true;
		cout<<"Turn Left"<<endl;
//		pidController.SetSetpoint(gyro.GetAngle());
		myRobot.MecanumDrive_Cartesian(0,0,-0.45,gyro.GetAngle());
	}

	void stopRobot()
	{
		Run = false;
		myRobot.MecanumDrive_Cartesian(0,0,0,gyro.GetAngle());
//		Wait(0.5);
		cout<<"Stop"<<endl;
	}
	void lowClaw_Open ()
	{
		lowClawSolenoid.Set(DoubleSolenoid::kForward);
		isLowClawClosed = false;
	}
	void highClawOpen()
	{
		highClawSolenoid.Set(DoubleSolenoid::kForward);
		isHighClawClosed =false;
	}

	void lowclaw_Close()
	{
		lowClawSolenoid.Set(DoubleSolenoid::kReverse);
		isLowClawClosed = true;
	}
	void highClawClose()
	{
		highClawSolenoid.Set(DoubleSolenoid::kReverse);
		isHighClawClosed = true;
	}

	void LeftAlerLightStatus(bool state )
	{
		leftAlertLight.Set(state);
	}
	void RightAlertLightStatus(bool state)
	{
		RightAlertLight.Set(state);
	}
	void CombinedIntakeStatus(bool state)
	{
		if (state)
		{
			combinedIntake.Set(1);
		}
		else
		{
			combinedIntake.Set(0);
		}

	}
	void SideIntakeStatus(bool state)
	{
		if (state)
		{
			sideIntake.Set(1);
		}
		else
		{
			sideIntake.Set(0);
		}
	}
	void ShooterStatus (bool state)
	{
		if (state)
		{
			shooter.Set(1);
		}
		else
		shooter.Set(0);
	}
	void DrivePIDStatus(bool state)
	{
		if (state)
		{
			pidController.Disable();
		}
		else
		{
			pidController.Enable();

		}
	}
	void hook_down()
	{
		this->liftMotor.Set(-1.0);
		Wait(0.1);
		this->liftMotor.Set(0);
	}

	void hook_down(double time_s)
	{
		this->liftMotor.Set(-1.0);
		Wait(time_s);
		this->liftMotor.Set(0);
	}

	void hook_up()
	{
		this->liftMotor.Set(1.0);
		Wait(0.1);
		this->liftMotor.Set(0);
	}

	bool is_hook_down()
	{
		return !this->m_lower_limit.Get();
	}

	bool is_hook_up()
	{
		return !this->m_upper_limit.Get();
	}
	bool is_rotor_taken()
	{
		return !m_rotor_limit.Get();
	}
	bool is_reached_wall()
	{
		return !m_front_limit.Get();
	}
	void set_m_updown(double speed)
	{
		//speed = -1.0 *speed;
		this->liftMotor.Set(speed);
	}
	void set_m_down(double speed)
	{
		speed =1.0 *speed;
		this->liftMotor.Set(speed);
	}
	void makesure_hook_is_up()
	{
		while(!is_hook_up())
		{
			hook_up();
		}
	}

	void printOnTheConsole()
	{
		currentDistance = ultra.GetRangeMM();
		currentGytoAngle = gyro.GetAngle();

		cout<<"Distance: "<<currentDistance<<endl;
		cout<<"Gyro Angle: "<<currentGytoAngle<<endl;
		cout<<"Blocks[0].x: "<<blocks[0].x<<endl;
		cout<<"Blocks[1].x: "<<blocks[1].x<<endl;

		cout<<"Blocks->x: "<<blocks->x<<endl;

		myRobot.SetSafetyEnabled(false);
		uint16_t blah = getBlocks(100);
		printf("blocks: ");printf("%d", blah);printf("\n"); //prints number of block to the console
		blocks[0].print();// prints x, y, width, and etc. to the console (the vairables in the block object)
		printf("\n");//new line(space)
	}

	void allSmartDashboard()
	{
		SmartDashboard::PutBoolean("Forward",Forward);
		SmartDashboard::PutBoolean("Backward",Backward);
		SmartDashboard::PutBoolean("Left",Left);
		SmartDashboard::PutBoolean("Turn Left",TurnLeft);
		SmartDashboard::PutBoolean("Right",Right);
		SmartDashboard::PutBoolean("Turn Right",TurnRight);
		SmartDashboard::PutBoolean("Run", Run);
		SmartDashboard::PutBoolean("Vision On", isVisionOn);
		SmartDashboard::PutBoolean("Two object", isTwo);
		SmartDashboard::PutBoolean("Low Claw Closed", isLowClawClosed);
		SmartDashboard::PutBoolean("High Claw Closed", isHighClawClosed);
		SmartDashboard::PutBoolean("Light", isLighten);
		SmartDashboard::PutNumber("Object Size:",objectSize);
		SmartDashboard::PutBoolean("Roller:",isRollerRan);
		SmartDashboard::PutBoolean("Shooter:",isShooterRan);
		SmartDashboard::PutBoolean("Side Intake",isSideIntakeRan);
		SmartDashboard::PutBoolean("High Limit Detected",isHighReached);
		SmartDashboard::PutBoolean("Low Limit Detected",isLowReached);
		SmartDashboard::PutBoolean("Drive PID",isPIDisabled);

		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		SmartDashboard::PutData("Gyro Angle:",&gyro);

		SmartDashboard::PutNumber("X: ", blocks->x);
		SmartDashboard::PutNumber("Y: ", blocks->y);
		SmartDashboard::PutNumber("Width: ", blocks->width);
		SmartDashboard::PutNumber("Distance: ",currentDistance );
		SmartDashboard::PutNumber("X Acc:",accleroMeter.GetX());
		SmartDashboard::PutNumber("Y Acc:",accleroMeter.GetY());
		SmartDashboard::PutNumber("Z Acc:",accleroMeter.GetZ());

	}

	void caculateTime()
	{
		long i = 10000000L;
		clock_t start, finish;
		double duration;
		cout<<"Duration: "<<i<<endl;
		start = clock();
		while(i--);
		finish = clock();
		duration = (double)(finish-start)/CLOCKS_PER_SEC;
		cout<<duration<<" seconds"<<endl;
		system("pause");
	}
//	    bool isGyroReset (gyroState)
//	    {
//
//
//	    }

	void RobotInit() override {
		// invert the left side motors
		// you may need to change or remove this to match your robot
		i2c = new I2C(I2C::Port::kOnboard, PIXY_I2C_DEFAULT_ADDR);//(I2C::Port::kOnboard or kMXP, Pixy Address)
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor,true);
		SmartDashboard::init();
		shooter.SetInverted(true);
		roller.SetInverted(true);
		ultra.SetAutomaticMode(true);
		ultra.SetDistanceUnits(Ultrasonic::kMilliMeters);
//		frc::LiveWindow::GetInstance();

//		pidControllerUltra->SetSetpoint(500);
//		pidControllerUltra->SetAbsoluteTolerance(10);
//		//		pidControllerUltra.SetInputRange(300, 1000);
//		//		pidControllerUltra.SetContinuous(true);
//		pidControllerUltra->Enable();// begin PID control

		LiveWindow::GetInstance()->AddActuator("DriveSystem", "RobotDrive",
				pidController);
		LiveWindow::GetInstance()->AddActuator("DriveSystem", "UltraSonic",
				pidControllerUltra);
		LiveWindow::GetInstance()->AddActuator("DriveSystem", "Vision",
				pidControllerVision);

//		ultra = new Ultrasonic(0,1);

		/*  panServoPos = panServoCenter;
		 double pspT = panServoPos*ms;
		 panServo.Set(panServoPos);
		 tiltServoPos = tiltServoCenter;
		 double tspT = tiltServoPos*ms;
		 tiltServo.Set(tspT);
		 Test1- Make the Servo stay in the middle.*/

		panServoPos = panServoCenter;
		tiltServoPos = tiltServoCenter;

		panServo.SetAngle(panServoPos);
		tiltServo.SetAngle(tiltServoCenter);//Test 2
		gyro.Reset();
		compressor.Start();

		pidController.SetSetpoint(-2);
		pidController.SetAbsoluteTolerance(2);
		pidController.SetInputRange(-180, 180);
		pidController.SetContinuous(true);
//		pidController.Enable();
		pidControllerVision->SetSetpoint(160);
		pidControllerVision->SetAbsoluteTolerance(5);
		pidControllerVision->SetInputRange(1, 319);
		pidControllerVision->SetContinuous(true);

		pidControllerUltra->SetSetpoint(400);
		pidControllerUltra->SetAbsoluteTolerance(10);
		pidControllerUltra->SetInputRange(100,1800);
		pidControllerUltra->SetContinuous(true);

		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		allSmartDashboard();
		makesure_hook_is_up();

		{
//			frc::VictorSP frontLeftMotor{0};
//			frc::VictorSP rearLeftMotor{1};
//			frc::VictorSP frontRightMotor{2};
//			frc::VictorSP rearRightMotor{3};
//			frc::VictorSP shooter {4};
//			frc::VictorSP roller {5};
//			frc::VictorSP liftMotor {6};
//			frc::VictorSP sideIntake {7};
//			frc::VictorSP combinedIntake {8};
//			frc::DoubleSolenoid lowClawSolenoid {0,1};
//			frc::DoubleSolenoid highClawSolenoid {2,3};
			autoRecorder.AddDevice("Front Left",&frontLeftMotor);
			autoRecorder.AddDevice("Rear Left",&rearLeftMotor);
			autoRecorder.AddDevice("Front Right",&frontRightMotor);
			autoRecorder.AddDevice("Rear Right",&rearRightMotor);
			autoRecorder.AddDevice("Shooter",&shooter);
			autoRecorder.AddDevice("Lift",&liftMotor);
			autoRecorder.AddDevice("sideIntake",&sideIntake);
			autoRecorder.AddDevice("Combined Intake",&combinedIntake);
			autoRecorder.AddDevice("Low Claw Solenoid",&lowClawSolenoid);
			autoRecorder.AddDevice("High Claw Solenoid",&highClawSolenoid);
			CameraServer::GetInstance()->StartAutomaticCapture(1);

		}

	}

	void TeleopPeriodic() override {
		while(IsOperatorControl()&&IsEnabled())
		{
			pidControllerVision->Disable();
			pidControllerUltra->Disable();
//     	pidController.Enable();

			currentDistance = ultra.GetRangeMM();
			currentGytoAngle = gyro.GetAngle();

			cout<<"Distance: "<<currentDistance<<endl;
			cout<<"Gyro Angle: "<<currentGytoAngle<<endl;
			cout<<"Blocks[0].x: "<<blocks[0].x<<endl;
			cout<<"Blocks[1].x: "<<blocks[1].x<<endl;

			cout<<"Blocks->x: "<<blocks->x<<endl;


			uint16_t blah = getBlocks(100);
			printf("blocks: ");printf("%d", blah);printf("\n");//prints number of block to the console
			blocks[0].print();// prints x, y, width, and etc. to the console (the vairables in the block object)
			printf("\n");//new line(space)

			myRobot.SetSafetyEnabled(false);

			if (joystick.GetRawButton(1))
			{
				isPIDisabled = !isPIDisabled;
			}
			DrivePIDStatus(isPIDisabled);

			if (joystick.GetRawButton(2))
			{
				gyro.Reset();
			}
//
//		if (joystick.GetRawButton(4))
//		{
//			//			pidController.Enable();
//			pidControllerVision->Enable();
//
//		}

//		double mcorrectedRotate = joystick.GetRawAxis(0);
//		double mcorrectedInOut= joystick.GetRawAxis(1);
//		double correctedY =  mcorrectedInOut-mcorrectedRotate;
//		myRobot.MecanumDrive_Cartesian(joystick.GetRawAxis(4),joystick.GetRawAxis(1),turn,gyro.GetRate());

//		double acceptableX=joystick.GetRawAxis(4);
			double acceptableX=joystick.GetRawAxis(4);
			if(fabs(acceptableX)<0.2)
			{
				acceptableX = 0;

			}

			double acceptableY = joystick.GetRawAxis(1);
			if (fabs(acceptableY) < 0.2)
			{
				acceptableY = 0;

			}

			double turn = 0;
			if(joystick.GetRawAxis(2))
			{
//			gyro.Reset();
				pidController.SetSetpoint(0);
				turn = - (joystick.GetRawAxis(2));
//			pidController.Disable();
//			pidController.SetSetpoint(gyro.GetAngle());

				myRobot.MecanumDrive_Cartesian(0,0,turn);

			}
			if (joystick.GetRawAxis(3))
			{
//			gyro.Reset();

				pidController.SetSetpoint(0);
				turn = (joystick.GetRawAxis(3));
//			pidController.Disable();

//			pidController.SetSetpoint(gyro.GetAngle());
				myRobot.MecanumDrive_Cartesian(0,0,turn);

			}
			// begin PID control

			double acceptableZ = joystick.GetRawAxis(2);
			if (fabs(acceptableZ) < 0.2)
			{
				acceptableZ = 0;
			}
			if(joystick.GetRawButton(3))
			{
				myRobot.MecanumDrive_Cartesian(-acceptableX,-acceptableY,-turn,gyro.GetAngle());
			}
			else
			{
				myRobot.MecanumDrive_Cartesian(acceptableX,acceptableY,turn,gyro.GetAngle());
			}

			if (joystick.GetRawButton(3))
			{
				centerServo();
			}

			if (joystick.GetRawButton(5))
			{
				followBlock();
			}
			if (joystick.GetRawButton(1))
			{
				loopTrackWithPanTiltServo();
			}

			//Claw Function
			if (joystick2.GetRawButton(4))
			{

				highClawClose();
			}
			else
			{
				highClawOpen();
			}
			if (joystick2.GetRawButton(3))
			{
				lowclaw_Close();
			}
			else
			{
				lowClaw_Open();
			}

			if (joystick2.GetRawButton(5))
			{
				isCombinedIntakeRan = !isCombinedIntakeRan;
			}
			CombinedIntakeStatus(isCombinedIntakeRan);

			if (joystick2.GetRawButton(6))
			{
				isSideIntakeRan = !isSideIntakeRan;

				if(joystick2.GetRawButton(8))
				{
					isLighten = !isLighten;
				}

				LeftAlerLightStatus(isLighten);
				RightAlertLightStatus(isLighten);

			}
			SideIntakeStatus(isSideIntakeRan);
			double liftMotorspeed = joystick2.GetRawAxis(1);
//		if (joystick2.GetPOV(1))
//		{
//			liftMotorspeed = 1;
//		}
//		else
//		{
//			liftMotorspeed = 0;
//		}
//		if (joystick2.GetPOV(4))
//		{
//			liftMotorspeed = -1;
//		}
//		else
//		{
//			liftMotorspeed = 0;
//		}

			if(this->is_hook_down())
			{
				isLowReached = true;
				if(liftMotorspeed < 0)
				{
					liftMotorspeed = 0;
				}
			}

			if(this->is_hook_up())
			{
				isHighReached = true;
				if(liftMotorspeed > 0)
				{
					liftMotorspeed = 0;
				}
			}
			liftMotor.Set(liftMotorspeed);

			if (joystick2.GetRawButton(1))
			{
				roller.Set(0.9);		//**
			}
			else
			{
				roller.Set(0);
			}

			if(joystick2.GetRawButton(7))	//shooter requires roller spins
			{
				isShooterRan = !isShooterRan;
			}
			ShooterStatus(isShooterRan);
			if (joystick.GetRawButton(10))
			{
				mac->Record();
			}
		}

		mac->WriteFile("/var/frc/1.csv");
	}

//	void AutonomousInit() override
//	{
//
//		timer.Reset();
//
////		pidControllerVision->Reset();
////		pidController.Reset();
////		pidControllerUltra->Reset();
//		pidControllerUltra->SetSetpoint(500);
//		pidControllerUltra->SetAbsoluteTolerance(5);
//		pidControllerUltra->SetInputRange(1,3000);
//		pidControllerUltra->SetContinuous(true);
//		pidController.SetSetpoint(0);
//		pidController.SetAbsoluteTolerance(5);
////		pidController.SetInputRange(-180, 180);
//		pidController.SetContinuous(true);
////		pidController.Enable();
//		pidControllerVision->SetSetpoint(160);
//		pidControllerVision->SetAbsoluteTolerance(5);
//		pidControllerVision->SetInputRange(1, 319);
//		pidControllerVision->SetContinuous(true);
//
//	}
//	void AutonomousPeriodic() override
//	{
//		printOnTheConsole();
//
//		if (!pidControllerVision->OnTarget())
//		{
//			pidControllerVision->Enable();
//			cout<<"Vision Enable"<<endl;
//
//		}
//		else
//		{
//
//			pidControllerVision->Disable();
////			pidController.Enable();
//			cout<<"Vision Disabled"<<endl;
//		}
//
//		if (!pidControllerUltra->OnTarget())
//		{
////			pidController.Enable();
//
//			if (pidControllerVision->OnTarget())
//			{
//				cout<<"Ultra Enabled"<<endl;
//				pidControllerUltra->Enable();
//			}
//
//		}
//		else
//		{
//			pidControllerUltra->Disable();
//
//			cout<<"Ultra Disabled"<<endl;
//		}
////
////		if (!pidController.OnTarget())
////		{
////			pidController.Enable();
////		}
////		else
////		{
////			pidController.Disable();
////			cout<<"Drive Disabled"<<endl;
////		}
//
//	}
	void AutonomousPeriodic() override
	{

		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoPlayReordedFile) {
			mac->ReadFile("var/frc/1.csv");
			if (!mac->IsFinished())
			{
				mac->PlayBack();
			}
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);

		}
		if (autoSelected == autoNameCustom)
		{
			std::cout << "Running Custom Autonomous" << std::endl;
			timer.Start();
			auto end = timer.Get();
			if(end<=4)	// 2s y y // 4s f y // 5s y y
			{
				if(!is_reached_wall())
				{
					goForward();
				}
				else
				{
					stopRobot();
				}
			}
			else
			{
				stopRobot();
			}

		}
		if (autoSelected == autoPIDCombined)
		{
			cout<<"Running PIDCombined Autonomous"<<endl;
			myRobot.SetSafetyEnabled(false);
			pidController.SetPID(0.02,0,0);
			pidControllerVision->SetInputRange(1,319);
			pidControllerVision->SetSetpoint(160);
			pidControllerVision->SetAbsoluteTolerance(10);

			if(!pidControllerVision->OnTarget())
			{
				pidControllerVision->Enable();
			}
			else
			{
				pidControllerVision->Disable();
			}

			pidControllerUltra->SetInputRange(1,2500);
			pidControllerUltra->SetSetpoint(10);
			pidControllerUltra->SetAbsoluteTolerance(10);
			if(!pidControllerUltra->OnTarget())
			{
				pidControllerUltra->Enable();
			}
			else
			{
				pidControllerUltra->Disable();
			}

		}

		if (autoSelected == autoNameDefault) {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);

		}

	}
	void TestInit()override
	{

		pidControllerUltra->SetSetpoint(500);
		pidControllerUltra->SetAbsoluteTolerance(5);
		pidControllerUltra->SetInputRange(1,3000);
		pidControllerUltra->SetContinuous(true);
		pidController.SetSetpoint(0);
		pidController.SetAbsoluteTolerance(5);
		//		pidController.SetInputRange(-180, 180);
		pidController.SetContinuous(true);
		//		pidController.Enable();
		pidControllerVision->SetSetpoint(160);
		pidControllerVision->SetAbsoluteTolerance(5);
		pidControllerVision->SetInputRange(1, 319);
		pidControllerVision->SetContinuous(true);

	}
	void TestPeriodic()
	{
//	    	Tele();
//	    	followBlock();

//TeleopPeriodic();
//loopTrackWithPanTiltServo();
		currentDistance = ultra.GetRangeMM();
		currentGytoAngle = gyro.GetAngle();

		cout<<"Distance: "<<currentDistance<<endl;
		cout<<"Gyro Angle: "<<currentGytoAngle<<endl;
		double first = blocks[0].x;
		double second = blocks[1].x;
		double currentX =( first+second)/2;
		cout<<"First: "<<first<<endl;
		cout<<"Second: "<<second<<endl;
		cout<<"Current X: "<<currentX<<endl;
		uint16_t blah = getBlocks(100);
		printf("blocks: ");printf("%d", blah);printf("\n");//prints number of block to the console
		blocks[0].print();// prints x, y, width, and etc. to the console (the vairables in the block object)
		printf("\n");//new line(space)
//		if (!pidControllerUltra->OnTarget())
//		{
//			pidController.Enable();
//			pidControllerUltra->Enable();
//		}
//		else
//		{
//			pidControllerUltra->Disable();
//		}
//				loopTrackWithPanTiltServo();

	}

private:
	class MyPIDOutput: public frc::PIDOutput {
	public:
		MyPIDOutput(frc::RobotDrive& r) :
		rd(r) {
			rd.SetSafetyEnabled(false);
		}

		void PIDWrite(double output) override {
//				rd.Drive(output*0.8, output*0.5); // write to myRobot (RobotDrive) by reference
			rd.MecanumDrive_Cartesian(0,0,.5*output);
		}
	private:
		frc::RobotDrive& rd;
	};

	class MyPIDInputUltra: public frc::PIDSource
	{
		Ultrasonic *ultra;
	public:
		MyPIDInputUltra(Ultrasonic *ul) {
			ultra = ul;
		}

		virtual double PIDGet() {
			//				rd.Drive(output*0.8, output*0.5); // write to myRobot (RobotDrive) by reference
			return ultra->GetRangeMM();
		}
	};

	class MyPIDOutPutUltra: public PIDOutput
	{
		RobotDrive *myRobot;
	public:
		MyPIDOutPutUltra(RobotDrive *mr)
		{
			myRobot = mr;
		}
		void PIDWrite(double output) override
		{
			myRobot->MecanumDrive_Cartesian(0,0.8*output,0);
		}
	};

	class MyPIDOutputVision: public PIDOutput
	{
		RobotDrive *myRobot;
	public:
		MyPIDOutputVision(RobotDrive *mr)
		{
			myRobot = mr;
		}
		void PIDWrite(double output) override
		{
			myRobot->MecanumDrive_Cartesian(-0.8*output,0,0);
		}
	};

	class MyPIDInputVision: public PIDSource
	{
		Block *blocks;
	public:
		MyPIDInputVision(Block *bs)
		{
			blocks =bs;
		}
		virtual double PIDGet()
		{
			return blocks->x;
		}
	};

	// internal class to write to myRobot (a RobotDrive object) using a PIDOutput

	// Gyro calibration constant, may need to be adjuste
	// Gyro value of 360 is set to correspond to one full revolution

	double kP = 0.04;
	double kI = 0;
	double kD = 0;

	double kUltraP = 0.0020;
	double kUltraI = 0;
	double kUltraD = 0;

	double kVisionP = 0.003;
	double kVisionI = 0;
	double kVisionD = 0;

	bool isVisionOn = false;
	int objectSize = 0;
	bool Forward = false;
	bool Backward = false;
	bool Left = false;
	bool Right = false;
	bool TurnLeft = false;
	bool TurnRight = false;
	bool Run = false;

	static constexpr int kFrontLeftMotorPort = 0;
	static constexpr int kRearLeftMotorPort = 1;
	static constexpr int kFrontRightMotorPort = 2;
	static constexpr int kRearRightMotorPort = 3;
	static constexpr int kJoystickPort = 0;

	frc::RobotDrive myRobot {kFrontLeftMotorPort, kRearLeftMotorPort,
		kFrontRightMotorPort, kRearRightMotorPort};

	frc::VictorSP frontLeftMotor {0};
	frc::VictorSP rearLeftMotor {1};
	frc::VictorSP frontRightMotor {2};
	frc::VictorSP rearRightMotor {3};
	frc::VictorSP shooter {4};
	frc::VictorSP roller {5};
	frc::VictorSP liftMotor {6};
	frc::VictorSP sideIntake {7};
	frc::VictorSP combinedIntake {8};

	Recorder autoRecorder {};

	Macro* mac;

	frc::DoubleSolenoid lowClawSolenoid {0,1};
	frc::DoubleSolenoid highClawSolenoid {2,3};

	frc::ADXRS450_Gyro gyro {SPI::kOnboardCS0};
	frc::Joystick joystick {kJoystickPort};
	frc::Joystick joystick2 {1};
	frc::BuiltInAccelerometer accleroMeter{Accelerometer::Range::kRange_8G};
	frc::PIDController pidController {kP, kI, kD, &gyro,
		new MyPIDOutput(myRobot)};
	MyPIDInputVision *myPidInpuVision = new MyPIDInputVision(blocks);
	PIDController *pidControllerVision = new PIDController(kVisionP,kVisionI,kVisionD,myPidInpuVision,new MyPIDOutputVision(&myRobot));

	MyPIDInputUltra *myPidInputUltra = new MyPIDInputUltra(&ultra);
	PIDController *pidControllerUltra = new PIDController(kUltraP,kUltraI,kUltraD,myPidInputUltra,new MyPIDOutPutUltra(&myRobot));
//	frc::PIDController pidControllerUltra {kultraP, kultraI, kultraD, MyPIDInputUltra,
//		new MyPIDOutputUltra(myRobot)};

	BlockType blockType;// it is the enum on the top
	bool skipStart;//skips to check 0xaa55, which is byte that tells pixy it is start of new frame
	uint16_t blockCount;//How many signatured objects are there?
	uint16_t blockArraySize;//not used in the code
	Block blocks[100];//array that stores blockCount array
	I2C* i2c;
	frc::Servo panServo {9};
	frc::Servo tiltServo {11};
	bool gyroState = false;
	bool isTwo =false;
	bool isLighten = false;
	bool isCombinedIntakeRan = false;
	bool isSideIntakeRan= false;
	bool isShooterRan = false;
	bool isRollerRan = false;
	bool isHighReached = false;
	bool isLowReached = false;
	bool isPIDisabled = false;

	const int panServoLeft = 0; //1500;
	const int panServoCenter = 110;//1000;
	const int panServoRight = 180;//700;
	int panServoPos;
	const int tiltServoUp = 0;//2300#900;
	const int tiltServoCenter = 90;
	const int tiltServoDown = 180;//1300;
	int tiltServoPos;
	frc::Ultrasonic ultra {0,1};
	Compressor compressor;
	Solenoid leftAlertLight {4};
	Solenoid RightAlertLight {5};
	frc::DigitalInput m_upper_limit {0};
	frc::DigitalInput m_lower_limit {1};
	frc::DigitalInput m_rotor_limit {2};
	frc::DigitalInput m_front_limit {3};
	bool isLowClawClosed = false;
	bool isHighClawClosed = false;

	Timer timer;
	double currentDistance;
	double currentGytoAngle;
	static constexpr double kValueToInches = 0.125;
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "Touch Down";
	const string autoPlayReordedFile = "Play Recorded File";
	const string autoPIDCombined = "Vision & Ultrasonic & Gyro PID";

//    lw=LiveWindow::GetInstance();
//	LiveWindow.AddActuator("drivetrain", "Drivetrain", );
};

START_ROBOT_CLASS(Robot)
