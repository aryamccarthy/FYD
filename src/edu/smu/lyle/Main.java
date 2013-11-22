package edu.smu.lyle;

import rxtxrobot.*;
import java.util.*;
public class Main {

	// Arm Angles
	static final int LIMBO_ANGLE = 40;

	static final int IR_DISPARITY = 172;
	// Pins
	static final int LOW_IR = 0;
	static final int HIGH_IR = 1;
	static final int BUMPER_RIGHT_PIN = 11;
	static final int BUMPER_LEFT_PIN = 12;
	static final int PH_PIN = 3;
	static final int UP_IR = 2;
	static final int TURBIDITY_PIN = 4;
	// Motors
	static final int LEFT_MOTOR = RXTXRobot.MOTOR1;   // Pin 5
	static final int RIGHT_MOTOR = RXTXRobot.MOTOR2;  // Pin 6
	static final int PUMP = RXTXRobot.MOTOR3;         // Pin 7
	static final int MIXER = RXTXRobot.MOTOR4;        // Pin 8
	static final int SERVO = RXTXRobot.SERVO1;        // Pin 10
	// Movement constants
	public static final double MOTOR_CONSTANT;        // BIGGER NUMBERS -> MORE LEFT SKEW
	static {
		MOTOR_CONSTANT = 0.94;
	}
	static final int MAX_SPEED = -450;
	static final int FOOT_TO_CLICK_CONVERSION = 268;   // BIGGER NUMBER -> FARTHER DISTANCE
	static final int TURN_CONSTANT = 330;//258              // BIGGER NUMBER -> BIGGER ANGLE
	// Dispensing constants.
	static final double SOLUTION_PH;
	static {
		SOLUTION_PH = 5.0;                      // TYPE WHATEVER THEY TELL YOU.
	}
	// Connection constants.
	static final String ROBOT_PORT = "/dev/tty.usbmodem14131";            //DEPENDS ON YOUR COMPUTER
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
	static boolean isLimbo;
	static boolean isMaze;
	static boolean isGap;

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
		for (int i = -2; i < minIndex; i++) {
			rightAngleTurn(Direction.LEFT);
		}
	}
	static void lookForRFIDwithTurns() {
		RFIDSensor sensor = new RFIDSensor();
		sensor.setPort(RFID_PORT);
		sensor.connect();
		runMotorsIndefinitely();
		while (true) {
			if (sensor.hasTag()) {
				stopMotors();
				break;
			}
			if (bothBumpersPressed()) {
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

	// END OF REQUIREMENTS *****************************************************************************/

	/*************BELOW THIS LINE DON'T FUCK WITH ME.**************************************************/

	/*************MOBILITY*****************************************************************************/

	static void driveThisManyFeet(double feet) {
		moveTheMotors((int)(FOOT_TO_CLICK_CONVERSION * feet));
		robot.resetEncodedMotorPosition(LEFT_MOTOR);
		robot.resetEncodedMotorPosition(RIGHT_MOTOR);
	}
	static void lessThanRightAngleTurn(Direction direction) {
		runMotorsToTurn(direction, TURN_CONSTANT);
	}
	static void runMotorsIndefinitely() {
		robot.runMotor(LEFT_MOTOR, (int)(MAX_SPEED/MOTOR_CONSTANT), RIGHT_MOTOR, MAX_SPEED, 0);
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
	static enum Direction {LEFT, RIGHT}
	static void runMotorsToTurn(Direction direction, int duration) {
		int dir = (direction == Direction.RIGHT) ? 1 : -1;
		robot.runEncodedMotor(LEFT_MOTOR, dir * (int) (MAX_SPEED), duration, RIGHT_MOTOR, -1 * dir * MAX_SPEED, duration);
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
		final double MAX_VOLUME = 2.55;   // Most that can be dispensed in MAX_TIME at MAX_SPEED.
		final double VOLUME_TO_TIME = MAX_TIME/MAX_VOLUME;
		double time = milliliters * VOLUME_TO_TIME;
		echo(time);
		for (int i = 0; i < (int)time/MAX_TIME; i++)
			robot.runMotor(PUMP, -1 * MAX_SPEED, MAX_TIME);
		echo(((int) time) % MAX_TIME);
		robot.runMotor(PUMP, -1 * MAX_SPEED, ((int)time) % MAX_TIME);
	}
	static void stopPump() {
		robot.runMotor(PUMP, 0, 0);
	}
	static void preLoad() {
		echo("Running.");
		robot.runMotor(PUMP, -1 * MAX_SPEED, 0);
		echo("Motor running.");
		robot.sleep(300);
		waitForBump();
		echo("Bumped.");
		stopPump();
	}
	static void remediate() {
		robot.runMotor(MIXER, 75, 0);
		int temperature = pollTemperature();
		echo("Initial temperature: " + temperature);
		double turbidity = calculateTurbidity(pollTurbidityPin());
		echo("Turbidity: " + turbidity);
		double tankPH = calculatePH(pollPhPin(), temperature);
		echo("Initial pH: " + tankPH);
		while (7.0 < tankPH || tankPH < 7.5) {
			if (SOLUTION_PH < 7.0 && tankPH < 7.0)
				break;
			if (SOLUTION_PH > 7.5 && tankPH > 7.5)
				break;
			dispenseRemediationFluidVolume(millilitersToDispense(tankPH, SOLUTION_PH));
			robot.sleep(4000);
			tankPH = calculatePH(pollPhPin(), temperature);
			echo("pH is now: " + tankPH);
		}
		robot.runMotor(MIXER, 0, 0);
		setServoAngle(0);
	}

	/*************NAVIGATION**************************************************************************/
	static int getSensorData(int pinNumber) {
		robot.refreshAnalogPins();
		return robot.getAnalogPin(pinNumber).getValue();
	}
	static int getDigitalData(int pinNumber) {
		robot.refreshDigitalPins();
		return robot.getDigitalPin(pinNumber).getValue();
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
	static boolean IRDisparity() {
		int sumLow = 0;
		int sumHigh = 0;
		int numGoodReads = 0;
		do {
			robot.refreshAnalogPins();
			sumLow += robot.getAnalogPin(LOW_IR).getValue();
			sumHigh += robot.getAnalogPin(HIGH_IR).getValue();
			numGoodReads++;
		} while (numGoodReads < 5);
		return Math.abs((sumHigh - sumLow)/numGoodReads) > IR_DISPARITY;
	}
	static boolean bumperPressed(Direction direction) {
		int bumperValue = (direction == Direction.LEFT) ? getDigitalData(BUMPER_LEFT_PIN) : getDigitalData(BUMPER_RIGHT_PIN);
		return bumperValue == 0;
	}
	static boolean onlyOneBumperPressed(Direction direction) {
		robot.refreshDigitalPins();
		return robot.getDigitalPin((direction == Direction.LEFT) ? BUMPER_LEFT_PIN : BUMPER_RIGHT_PIN).getValue() == 0 &&  !(robot.getDigitalPin((direction == Direction.RIGHT) ? BUMPER_LEFT_PIN : BUMPER_RIGHT_PIN).getValue() == 0);
	}
	static boolean eitherBumperPressed() {
		robot.refreshDigitalPins();
		return robot.getDigitalPin(BUMPER_LEFT_PIN).getValue() == 0 || robot.getDigitalPin(BUMPER_RIGHT_PIN).getValue() == 0;
		//return (getDigitalData(BUMPER_LEFT_PIN) < BUMPER_THRESHOLD || getDigitalData(BUMPER_RIGHT_PIN) < BUMPER_THRESHOLD);
	}
	static boolean bothBumpersPressed() {
		robot.refreshDigitalPins();
		return robot.getDigitalPin(BUMPER_LEFT_PIN).getValue() == 0 && robot.getDigitalPin(BUMPER_RIGHT_PIN).getValue() == 0;
	}
	static void waitForBump() {
		// 1023 is pressed. Each motor has a threshold for when it's "pressed".
		while (true) if (bothBumpersPressed())
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
	static int pollPhPin() {
		return pollSensorData(PH_PIN);
	}
	static int pollTurbidityPin() {
		return pollSensorData(TURBIDITY_PIN);
	}
	static double calculateTurbidity(double input) {
		// Relationship: Turb = 4 + 132*(3-V) (Arbitrary)
		final double TURBIDITY_PIN_MAX_VALUE = 100.0;
		return 726.121-2.7886 * input;
		/*voltage = 3 - voltage;
		voltage *= 132;
		voltage += 4;
		return voltage;*/
	}
	static double calculatePH(double input, double temperature) {
		// Relationship: E = E0 - G(RT/F)2.303pH
		//final double E0 = 1;
		//final double gain = 9.2;
		//final double R = 8.314;
		//final double nE = 1;
		//final double F = 96500;
		//temperature += 273.15;
		return 12.3128- (0.0000342684 * (temperature+273.15) * input);
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
		robot.setMixerSpeed(75);
		setServoAngle(0);
		robot.setResetOnClose(false);
	}
	static void cleanup(){
		setServoAngle(0);
		robot.close();
	}
	static<T> void echo(T arg) {
		System.out.println(arg);
	}

	static void limbo() {
		runMotorsIndefinitely();
		while (true) if (bothBumpersPressed())
			break;
		stopMotors();
		setServoAngle(LIMBO_ANGLE);
		pointInShortestDirection();
		driveThisManyFeet(10);
		setServoAngle(0);
	}

	static void maze() {  // Designed to run immediately after finding the RFID tag.
		for (int i = 0; i < 2; i++) {
			rightAngleTurn(Direction.RIGHT);
		}
		while (true) {
			runUntilBumped();
			moveBackwards(100);
			rightAngleTurn(Direction.RIGHT);
			robot.sleep(100);

		}
	}
	static int leftDistance;
	static int rightDistance;
	static void measureDistancesLeftAndRight() {
		//moveBackwards((int)(0.5 * FOOT_TO_CLICK_CONVERSION));
		leftDistance = pollPingDistance();
		rightAngleTurn(Direction.RIGHT);
		rightAngleTurn(Direction.RIGHT);
		rightDistance = pollPingDistance();
		rightAngleTurn(Direction.LEFT);
		rightAngleTurn(Direction.LEFT);
	}

	static double inchesToFeet (double inches) {
		return (double)inches / 12;
	}
	static double feetToInches (double feet) {
		return feet * 12;
	}
	static double inchesToCm (double inches) {
		return inches * 2.54;
	}
	static double cmToInches (double cm) {
		return cm / 2.54;
	}

	static void gapWithoutSideInformation() {
		runMotorsIndefinitely();
		while (true) {
			if (bothBumpersPressed()) {
				if (IRDisparity())
					break;
				else {
					rightAngleTurn(Direction.RIGHT);
					runMotorsIndefinitely();
				}
			}
			if (onlyOneBumperPressed(Direction.LEFT)) {
				lessThanRightAngleTurn(Direction.RIGHT);
				runMotorsIndefinitely();
			}
		}
		stopMotors();
		rightAngleTurn(Direction.RIGHT);
		runMotorsIndefinitely();
		while (robot.getPing() < 30)
			continue;
		stopMotors();
		rightAngleTurn(Direction.LEFT);
		driveThisManyFeet(4.0);

	}
	static void gap() {
		rightAngleTurn(Direction.LEFT);
		rightAngleTurn(Direction.LEFT);
		driveThisManyFeet(6.4);
		measureDistancesLeftAndRight();
		rightAngleTurn(leftDistance > rightDistance ? Direction.LEFT : Direction.RIGHT);
		driveThisManyFeet(inchesToFeet(cmToInches(270 - leftDistance > rightDistance ? rightDistance : leftDistance)));
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


	static final int ROBOT_WIDTH = 0;
	static void navigate() {
		pointInShortestDirection();
		runUntilBumped();
		measureDistancesLeftAndRight();
		int width = leftDistance + rightDistance + ROBOT_WIDTH;
		if (231 < width && width < 257)
			gap();
		else if (205 < width && width <= 231)
			maze_hole();
		else if (257 <= width && width <= 269)
			;//maze_wall();
		else if (330 <= width && width <= 390)
			;//far_wall();
		else assert (width > 390);
		;//test_side();
		;//seekWater();
		;//approachWater();
		remediate();
	}

	static void navigateObstacle () {
		if (isLimbo)
			limbo();
		else if (isGap)
			gap();
		else if (isMaze)
			maze();
	}

	static void doAllSix() {
		//identifyObstacle();
		navigateObstacle();
		seekWater();
		remediate();
		returnHome();
	}

	static void returnHome() {
		if (isLimbo) {

			setServoAngle(LIMBO_ANGLE);
			;
		}
	}

	static void RFIDAndMaze() {
		runAndCorrect();

	}

	static void maze_wall() {
		runMotorsIndefinitely();
		while (true) {
			if (bothBumpersPressed()) {
				if (IRDisparity())
					break;
				else {
					rightAngleTurn(Direction.LEFT);
					runMotorsIndefinitely();
				}
			}
			if (onlyOneBumperPressed(Direction.RIGHT)) {
				lessThanRightAngleTurn(Direction.LEFT);
				runMotorsIndefinitely();
			}
		}
		stopMotors();
		rightAngleTurn(Direction.LEFT);
		runUntilBumped();
		maze_hole();
	}

	static void correction () {
		runMotorsIndefinitely();
		while (true) {
			if (bothBumpersPressed()) {
				if (IRDisparity())
					break;
				else {
					rightAngleTurn(Direction.LEFT);
					runMotorsIndefinitely();
				}
			}
			if (onlyOneBumperPressed(Direction.RIGHT)) {
				lessThanRightAngleTurn(Direction.LEFT);
				runMotorsIndefinitely();
			}
		}
		stopMotors();
	}

	static final int UP_THRESHOLD = 50;
	static boolean isUnderLimboBar() {
		int ir_data = pollSensorData(UP_IR);
		return ir_data < 204;
	}

	/*
	 * Promises to go from wall with hole through hole and around to other size.
	 */
	static void maze_hole() {
		rightAngleTurn(Direction.RIGHT);
		runUntilBumped();
		moveBackwards((int)(0.25 * FOOT_TO_CLICK_CONVERSION));
		rightAngleTurn(Direction.RIGHT);
		runMotorsIndefinitely();
		while (true) if (robot.getPing() > 50)
			break;
		stopMotors();
		rightAngleTurn(Direction.LEFT);
		runMotorsIndefinitely();
		while (true) if (robot.getPing() > 50)
			break;
		robot.sleep(400);
		//stopMotors();
		rightAngleTurn(Direction.LEFT);
		driveThisManyFeet(1.5);
		rightAngleTurn(Direction.RIGHT);
	}

	static void runAndCorrect() {
		pointInShortestDirection();
		runUntilBumped();
		boolean RFIDfound = false;
		runMotorsIndefinitely();
		RFIDSensor rfid = new RFIDSensor();
		rfid.setPort(RFID_PORT);
		rfid.connect();
		while (!rfid.hasTag()) {
			if (onlyOneBumperPressed(Direction.LEFT)) {
				lessThanRightAngleTurn(Direction.RIGHT);
				runMotorsIndefinitely();
			}
			if (bothBumpersPressed()) {
				moveBackwards((int)(0.25 * FOOT_TO_CLICK_CONVERSION));
				rightAngleTurn(Direction.RIGHT);
				runMotorsIndefinitely();
			}
		}
		stopMotors();
		switch (rfid.getTag()) {
			case "66006C11F9E2":
				echo ("\nThe location is Dadaab\n" +
						"The obstacle is a limbo bar.\n" +
						"The challenge is an elevated well.\n");
				break;
			case "66006C432D64":
				echo ("\nThe location is Fish Town.\n" +
						"The obstacle is a a maze.\n" +
						"The challenge is a ground-level water basin.\n");
				break;
			case "66006C001F15":
				echo ("\nThe location is Ali Ade.\n" +
						"The obstacle is an opening in a wall.\n" +
						"The challenge is an underground well.\n");
				break;
			default:
				echo ("Unrecognized RFID tag.");
				break;
		}

		rfid.close();
	}

	static void turnLeftOrRightShortest () {
		rightAngleTurn(leftDistance < rightDistance ? Direction.LEFT : Direction.RIGHT);
	}

	/*~~~~~~~~~~~~~~~~~~~~~~~ WATER ARM THINGS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	static void seekWater() {
		seekWell();
		approachWell();
		lowerArmIntoWater();
	}
	static void approachWell () {
		rightAngleTurn(Direction.LEFT);
		runUntilBumped();
		moveBackwards((int)(0.25 * FOOT_TO_CLICK_CONVERSION));
	}
	static final int LEFT = 0;
	static final int RIGHT = 1;
	static int direction;
	static void seekWell() {
		if (isLimbo) {
			runUntilBumped();
			measureDistancesLeftAndRight();
			turnLeftOrRightShortest();
			runMotorsIndefinitely();
			while (!IRDisparity())
				continue;
			stopMotors();
		}
		runMotorsIndefinitely();
		int count = 0;
		while (true) {
			if (eitherBumperPressed()) {
				for (int i = 0; i < 2; i++) {
					rightAngleTurn(Direction.LEFT);
				}
				count++;
				count %= 2;
			}
			if (robot.getPing() < 150) {
				stopMotors();
				break;
			}
		}
	}
	static void lowerArmIntoWater() {
		int servoAngle = 0;
		while (!isWet()) {
			setServoAngle (servoAngle + 10);
			servoAngle += 10;
		}
	}
	static boolean isWet() {
		robot.refreshAnalogPins();
		return robot.getAnalogPin(5).getValue() > 100;
	}

	/*~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	public static void main(String[] args) {
		setup(); //~~~~~~~~~~~~~~~~DON'T MESS WITH THIS PART.
		doAllSix();
		cleanup(); //~~~~~~~~~~~~~~DON'T MESS WITH THIS PART.
	}
}
