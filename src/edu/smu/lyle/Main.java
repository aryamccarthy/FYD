package edu.smu.lyle;

import rxtxrobot.*;
import java.util.*;
public class Main {

	// Arm Angles
	static final int LIMBO_ANGLE = 40;

	static final int HIGH_ANGLE = 0;
	static final int MEDIUM_ANGLE = 110;
	static final int LOW_ANGLE = 140;
	// Pins
	static final int BUMPER_RIGHT_PIN = 1;
	static final int BUMPER_LEFT_PIN = 2;
	static final int PH_PIN = 3;
	static final int TURBIDITY_PIN = 4;
	static final int WATER_PIN = 5;
	// Motors
	static final int LEFT_MOTOR = RXTXRobot.MOTOR1;  // Pin 5
	static final int RIGHT_MOTOR = RXTXRobot.MOTOR2; // Pin 6
	static final int PUMP = RXTXRobot.MOTOR3;    // Pin 7
	static final int MIXER = RXTXRobot.MOTOR4;  // Pin 8
	static final int SERVO = RXTXRobot.SERVO1; // Pin 10
	// Movement constants
	public static final double MOTOR_CONSTANT;                // BIGGER NUMBERS -> MORE LEFT SKEW
	static {
		MOTOR_CONSTANT = 1.05;
	}
	static final int MAX_SPEED = -500;
	static final int FOOT_TO_CLICK_CONVERSION = 268;   // BIGGER NUMBER -> FARTHER DISTANCE
	static final int TURN_CONSTANT = 258;              // BIGGER NUMBER -> BIGGER ANGLE
	// Dispensing constants.
	static final double SOLUTION_PH;
	static {
		SOLUTION_PH = 5.0;                      // TYPE WHATEVER THEY TELL YOU.
	}
	// Connection constants.
	static final String ROBOT_PORT = "/dev/tty.usbmodem14131";           //DEPENDS ON YOUR COMPUTER
	static final String RFID_PORT = "/dev/tty.usbserial-A901JWCW";        //DEPENDS ON YOUR COMPUTER
	static RXTXRobot robot;
	// State variables (UNUSED)
	static String challenge;
	static {
		challenge = "";
	}
	static int temperature;
	static double pH;
	static double turbidity;

	// SPRINT 3 REQUIREMENTS ***************************************************************************

	static void square() {
		for (int i = 0; i < 4; i++) {
			driveThisManyFeet(2.0);
			robot.resetEncodedMotorPosition(LEFT_MOTOR);
			robot.resetEncodedMotorPosition(RIGHT_MOTOR);
			rightAngleTurn(Direction.LEFT);
			robot.resetEncodedMotorPosition(LEFT_MOTOR);
			robot.resetEncodedMotorPosition(RIGHT_MOTOR);
		}
	}

	static void moveThenHitAWallThenTurnLeft() {
		runUntilBumped();
		moveBackwards((int) (0.25 * FOOT_TO_CLICK_CONVERSION));
		rightAngleTurn(Direction.LEFT);
	}

	static void pointInShortestDirection() {
		ArrayList a = new ArrayList<Integer>();
		for (int i = 0; i < 4; i++) {
			int b = pollPingDistance();
			a.add(b);
			if (i != 3) rightAngleTurn(Direction.LEFT);
		}
		int minIndex = a.indexOf(Collections.min(a));
		echo(minIndex);
		for (int i = -1; i < minIndex; i++) {
			rightAngleTurn(Direction.LEFT);
		}
	}

	static void lookForRFIDwithTurns() {
		RFIDSensor sensor = new RFIDSensor();
		sensor.setPort(RFID_PORT);
		sensor.connect();
		runMotorsIndefinitely();
		while (true) {
			if (sensor.hasTag())
				break;
			if (eitherBumperPressed()) {
				stopMotors();
				rightAngleTurn(Direction.LEFT);
				runMotorsIndefinitely();
			}
		}

		String result = sensor.getTag();

		sensor.close();
		Main.challenge = result;
	}

	static void findRFID() {
		pointInShortestDirection();
		runUntilBumped();
		moveBackwards((int)(0.5 * FOOT_TO_CLICK_CONVERSION));
		rightAngleTurn(Direction.LEFT);
		lookForRFIDwithTurns();
	}

	static void findGap() {
		runMotorsIndefinitely();
		while (true) {
			if (eitherBumperPressed()) {
				if (pollPingDistance() > 30)
					break;
				else {
					stopMotors();
					rightAngleTurn(Direction.LEFT);
					runMotorsIndefinitely();
				}
			}
		}
		moveTheMotors((int)(6 * FOOT_TO_CLICK_CONVERSION));
		rightAngleTurn(Direction.RIGHT);
		moveTheMotors((int)(4 * FOOT_TO_CLICK_CONVERSION));
	}


	static enum ArmHeight {LOW, MEDIUM, HIGH}
	static void moveArm (ArmHeight height) {
		int angle = 0;
		if (height == ArmHeight.HIGH)
			angle = HIGH_ANGLE;
		if (height == ArmHeight.MEDIUM)
			angle = MEDIUM_ANGLE;
		if (height == ArmHeight.LOW)
			angle = LOW_ANGLE;
		setServoAngle(angle);
		assert isWet();
	}

	static void driveToGetRFID() {
		runMotorsIndefinitely();

		String result = keepCheckingForRFID();

		if (result.equals("66006C11F9E2")) result =
				"\nThe location is Dadaab\n" +
				"The obstacle is a limbo bar.\n" +
				"The challenge is an elevated well.\n";
		else if (result.equals("66006C432D64")) result =
				"\nThe location is Fish Town.\n" +
				"The obstacle is a a maze.\n" +
				"The challenge is a ground-level water basin.\n";
		else if (result.equals("66006C001F15")) result =
				"\nThe location is Ali Ade.\n" +
				"The obstacle is an opening in a wall.\n" +
				"The challenge is an underground well.\n";
		else result = "Undefined tag.";

		echo(result);
		stopMotors();

	}

	static void printAllTheData() {
		int temperature = pollTemperature();
		echo("Temperature: " + temperature);
		double turbidity = calculateTurbidity(pollTurbidityPin());
		echo("Turbidity: " + turbidity);
		double pH = calculatePH(pollPhPin(), temperature);
		echo("pH: " + pH);
		System.out.printf("%13s:%d\n%13s:%5.3f\n%13s:%5.3f", "Temperature", temperature, "Turbidity:", turbidity, "pH", pH);

	}

	static void remediateAndMix() {
		remediate();
		mix();
	}

	// END OF REQUIREMENTS *****************************************************************************/

	/*************BELOW THIS LINE DON'T FUCK WITH ME.**************************************************/

	/*************MOBILITY*****************************************************************************/

	static void driveThisManyFeet(double feet) {
		moveTheMotors(FOOT_TO_CLICK_CONVERSION * (int) feet);
		robot.resetEncodedMotorPosition(LEFT_MOTOR);
		robot.resetEncodedMotorPosition(RIGHT_MOTOR);
	}
	static void lessThanRightAngleTurn(Direction direction) {
		runMotorsToTurn(direction, TURN_CONSTANT*4/5);
	}
	static void runMotorsIndefinitely() {
		robot.runMotor(LEFT_MOTOR, (int)(MAX_SPEED/MOTOR_CONSTANT), RIGHT_MOTOR, MAX_SPEED, 0);
	}

	static void runSlowIndefinitely() {
		robot.runMotor(LEFT_MOTOR, (int)(MAX_SPEED/MOTOR_CONSTANT/50), RIGHT_MOTOR,MAX_SPEED/50, 0);
	}

	static void stopMotors() {
		robot.runMotor(LEFT_MOTOR, 0, RIGHT_MOTOR, 0, 0);
	}

	static void moveTheMotors(int ticks) {
		robot.runEncodedMotor(LEFT_MOTOR, (int)(MAX_SPEED), ticks, RIGHT_MOTOR, MAX_SPEED, ticks);
	}

	static void moveBackwards(int ticks) {
		robot.runEncodedMotor(LEFT_MOTOR, -1 * (int)(MAX_SPEED/MOTOR_CONSTANT), ticks, RIGHT_MOTOR, -1 *MAX_SPEED, ticks);
	}

	static void runUntilBumped() {
		runMotorsIndefinitely();
		waitForBump();
		stopMotors();
	}

	static enum  Direction {LEFT, RIGHT}
	static void runMotorsToTurn(Direction direction, int duration) {
		int dir = (direction == Direction.RIGHT) ? 1 : -1;
		robot.runEncodedMotor(LEFT_MOTOR, dir * (int) (MAX_SPEED /MOTOR_CONSTANT), duration, RIGHT_MOTOR, -1 * dir * MAX_SPEED, duration);
	}

	static void rightAngleTurn(Direction direction) {
		runMotorsToTurn(direction, TURN_CONSTANT); // 400 is arbitrary. Requires testing.
	}

	static void setServoAngle(int angle) {
		//robot.moveServo(SERVO, angle);
		robot.moveBothServos(angle, angle);
	}


	/*************REMEDIATION*************************************************************************/

	static void dispenseRemediationFluidVolume(double milliliters) {
		final int MAX_TIME = 30000; // 30 seconds
		final double MAX_VOLUME = 5.4;   // Most that can be dispensed in MAX_TIME at MAX_SPEED.
		final double VOLUME_TO_TIME = MAX_TIME/MAX_VOLUME;
		double time = milliliters * VOLUME_TO_TIME;
		echo(time);
		for (int i = 0; i < (int)time/MAX_TIME; i++)
			robot.runMotor(PUMP, MAX_SPEED, MAX_TIME);
		echo(((int) time) % MAX_TIME);
		robot.runMotor(PUMP, MAX_SPEED, ((int)time) % MAX_TIME);
	}

	static void stopPump() {
		robot.runMotor(PUMP, 0, 0);
	}

	static void preLoad() {
		echo("Running.");
		robot.runMotor(PUMP, MAX_SPEED, 0);
		echo("Motor running.");
		robot.sleep(300);
		waitForBump();
		echo("Bumped.");
		stopPump();
	}

	static void remediate() {
		int temperature = pollTemperature();
		echo("Initial temperature: " + temperature);
		double tankPH = calculatePH(pollPhPin(), temperature);
		echo("Initial pH: " + tankPH);
		while (7.0 < tankPH || tankPH < 7.5) {
			dispenseRemediationFluidVolume(millilitersToDispense(tankPH, SOLUTION_PH));
			mix();
			tankPH = calculatePH(pollPhPin(), temperature);
			echo("pH is now: " + tankPH);
		}
	}

	static void mix() {
		robot.runMixer(MIXER, 5000);
	}


	/*************NAVIGATION**************************************************************************/

	static int getSensorData(int pinNumber) {
		robot.refreshAnalogPins();
		return robot.getAnalogPin(pinNumber).getValue();
	}

	static int pollSensorData(int pinNumber) {
		int sum = 0;
		int numGoodReads = 0;
		do{
			int data = getSensorData(pinNumber);
			if (data >= 0 && data < 1024) {
				sum += data;
				numGoodReads++;
			}
		}while(numGoodReads < 10);
		return (int)Math.round((double)sum/numGoodReads);
	}

	static String keepCheckingForRFID() {
		RFIDSensor sensor = new RFIDSensor();
		sensor.setPort(RFID_PORT);
		sensor.connect();

		while(!sensor.hasTag()) sensor.sleep(100);
		String result = sensor.getTag();

		sensor.close();
		return result;
	}

	static int pollPingDistance() {
		int sum = 0;
		int numGoodReads = 0;
		do{
			int distance = robot.getPing();
			if (distance > 0 && distance < 1020) {
				sum += distance;
				numGoodReads++;
			}
		}while(numGoodReads < 10);
		return (int)Math.round((double)sum / numGoodReads);
	}

	static final int BUMPER_THRESHOLD = 900;

	static boolean bumperPressed(Direction direction) {
		robot.refreshAnalogPins();
		int bumperValue = (direction == Direction.LEFT) ? getSensorData(BUMPER_LEFT_PIN) : getSensorData(BUMPER_RIGHT_PIN);
		return bumperValue < BUMPER_THRESHOLD;
	}

	static boolean eitherBumperPressed() {
		robot.refreshAnalogPins();
		return (getSensorData(BUMPER_LEFT_PIN) < BUMPER_THRESHOLD || getSensorData(BUMPER_RIGHT_PIN) < BUMPER_THRESHOLD);
	}

	static boolean bothBumpersPressed() {
		return (getSensorData(BUMPER_LEFT_PIN) < BUMPER_THRESHOLD && getSensorData(BUMPER_RIGHT_PIN) < BUMPER_THRESHOLD);
	}

	static void waitForBump() {
		// 1023 is pressed. Each motor has a threshold for when it's "pressed".
		while (true) if (eitherBumperPressed())
			break;
	}


	/*************TESTING******************************************************************************/

	static int pollTemperature() {
		int sum = 0;
		int numGoodReads = 0;
		do{
			int temperature = robot.getTemperature();
			if (temperature > 0 && temperature < 100) {
				sum += temperature;
				numGoodReads++;
			}
		}while(numGoodReads < 10);
		return (int)Math.round((double)sum/numGoodReads);
	}

	static boolean isWet() {
		return getSensorData(WATER_PIN) < 100;
	}

	static int pollPhPin() {
		return pollSensorData(PH_PIN);
	}

	static int pollTurbidityPin() {
		return pollSensorData(TURBIDITY_PIN);
	}

	static double calculateTurbidity(double voltage) {
		// Relationship: Turb = 4 + 132*(3-V) (Arbitrary)
		final double TURBIDITY_PIN_MAX_VALUE = 100.0;
		return 5;
		/*voltage = 3 - voltage;
		voltage *= 132;
		voltage += 4;
		return voltage;*/
	}

	static double calculatePH(double input, double temperature) {
		// Relationship: E = E0 - G(RT/F)2.303pH
		final double E0 = 1;
		final double gain = 9.2;
		final double R = 8.314;
		final double nE = 1;
		final double F = 96500;
		temperature += 273.15;
		return 1195.17 - 96.5 * input;
		//return 14 - ((input-512)/96+7);//(E0 - voltage)/(gain * R * temperature / F / nE *2.303);
	}

	static double concentrationFromPH(double pH) {
		return Math.pow(10, -1*pH);
	}

	static double millilitersToDispense(double tankPH, double remediationPH) {
		double tankVolume = 1000;
		return concentrationFromPH(tankPH) * tankVolume / concentrationFromPH(remediationPH);
	}

	static void backupMix() {
		final int MOTOR_MIX_DISTANCE = 50;
		for (int i = 0; i < 5; i++) {
			moveBackwards(MOTOR_MIX_DISTANCE);
			moveTheMotors(MOTOR_MIX_DISTANCE);
		}

	}


	/*************MISCELLANEOUS************************************************************************/
	static void setup(){
		robot = new RXTXRobot();
		robot.setVerbose(true);
		robot.setPort(ROBOT_PORT);
		robot.connect();
		//robot.setMixerSpeed(250);
		//robot.sleep(10000);
		setServoAngle(0);
		robot.setResetOnClose(false);
	}
	static void cleanup(){
		setServoAngle(0);
		//robot.setResetOnClose(true);
		robot.close();
	}
	static<T> void echo(T arg) {
		System.out.println(arg);
	}


	/*~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	public static void main(String[] args) {
		setup(); //~~~~~~~~~~~~~~~~DON'T MESS WITH THIS PART.

				/*~~~~~~~~~ Mobility and Navigation ~~~~~~~~~~~~~~*/
		/*driveToGetRFID();
		moveThenHitAWallThenTurnLeft();
		square();
				/*~~~~~~~~~ Testing and Remediation ~~~~~~~~~~~~~~*/
		/*moveArm(ArmHeight.HIGH); // Can be HIGH, MEDIUM, or LOW.
		robot.sleep(10000);
		printAllTheData();
		remediateAndMix();
		int temperature = pollTemperature();
				for (int i = 0; i < 2; i ++)
						echo (calculatePH(pollPhPin(), pollTemperature()));


				/*~~~~~~~~~ DEBUGGING... ~~~~~~~~~~~~~~~~~~~~~~~~~*/
		/*echo ("pH OUTPUT: " + pollPhPin());
		preLoad();
		echo("TEMPERATURE: " + pollTemperature());
		echo("TURBIDITY OUTPUT: " + pollTurbidityPin());
		robot.runMixer(MIXER, 5000);
		driveThisManyFeet(3);
				for (int i = 0; i < 5; i++) {
						setServoAngle(20);
						setServoAngle(70);
				}
		waitForBump();
		preLoad();
		mix();
		dispenseRemediationFluidVolume(5.0);
		preLoad();
		backupMix();*/
		//rightAngleTurn(Direction.LEFT);
		//robot.runEncodedMotor(LEFT_MOTOR, MAX_SPEED, 40);
		//setServoAngle(90);
		//echo("Temperature: " + pollTemperature());
		//echo("pH: " + pollPhPin());
		//echo(pollPingDistance());
		//moveBackwards(300);
		//driveThisManyFeet(2);
		//runUntilBumped();
		//setServoAngle(100);
		//preLoad();
		//square();
		//pointInShortestDirection();
		//runUntilBumped();
		//runMotorsIndefinitely();
		//driveThisManyFeet(3.0);
		//echo (keepCheckingForRFID());
		//moveBackwards((int)(0.2*FOOT_TO_CLICK_CONVERSION));
		//lessThanRightAngleTurn(Direction.LEFT);
		//driveToGetRFID();
		//lookForRFIDwithTurns();
		//driveThisManyFeet(7);
		//runUntilBumped();
		//rightAngleTurn(Direction.RIGHT);
		//robot.runMotor(LEFT_MOTOR, MAX_SPEED, 10000);
		//robot.runMotor(LEFT_MOTOR, MAX_SPEED * -1, 10000);
		//runUntilBumped();
		//echo(keepCheckingForRFID());
		driveThisManyFeet(7);
		rightAngleTurn(Direction.RIGHT);
		driveThisManyFeet(4);
		cleanup(); //~~~~~~~~~~~~~~DON'T MESS WITH THIS PART.
	}
	static void slit() {
		while (true) {
			driveThisManyFeet(7.0);
			if (!eitherBumperPressed()) {
				rightAngleTurn(Direction.RIGHT);
				driveThisManyFeet(4.0);
				if (!eitherBumperPressed())
					break;
				else {
					moveBackwards((int)(0.1 * FOOT_TO_CLICK_CONVERSION));
					rightAngleTurn(Direction.LEFT);
				}
			}
		}
		echo("Made it through the slit!");
	}

	static void maze() {
		for (int i = 0; i < 2; i++) {
			rightAngleTurn(Direction.RIGHT);
		}
		while (true) {
			runUntilBumped();
			rightAngleTurn(Direction.RIGHT);

		}
	}
}
//
// HOLY SHIT RUNNING INTO WALLS MAKES US FACE THE WALL.