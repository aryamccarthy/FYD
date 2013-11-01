package edu.smu.lyle;

import rxtxrobot.*;
public class Main {

    // Pins
    static final int BUMPER_LEFT_PIN = 1;
    static final int BUMPER_RIGHT_PIN = 2; // Values range from 1023 (not pressed) to 0 (pressed)
    static final int WATER_PIN = 5;
    static final int TURBIDITY_PIN = 4;
    static final int PH_PIN = 3;

    // Motors
    static final int LEFT_MOTOR = RXTXRobot.MOTOR1;
    static final int RIGHT_MOTOR = RXTXRobot.MOTOR2;
    static final int PUMP = RXTXRobot.MOTOR3;    // Pin 7
    static final int MIXER = RXTXRobot.MOTOR4;  // Pin 8
    static final int SERVO = RXTXRobot.SERVO2; // Pin 10
    // Other
    static final int MAX_SPEED = 500;
    static final int FOOT_TO_CLICK_CONVERSION = 100;
    static final String ROBOT_PORT = "/dev/tty.usbmodemfa1431";           //DEPENDS ON YOUR COMPUTER
    static final String RFID_PORT = "/dev/tty.usbserial-A901JWCW";        //DEPENDS ON YOUR COMPUTER
    static RXTXRobot robot;
    static enum  Direction {LEFT, RIGHT}

    static String challenge;
    static int temperature;
    static double pH;
    static double turbidity;

    //sprint 3 mobility requirements

    public static void driveInTwoFootSquare() {
        final int clicks = 2 * FOOT_TO_CLICK_CONVERSION;
        for (int i = 0; i < 4; i++) {
            moveTheMotors(clicks);
            if (i != 3)
                rightAngleTurn(Direction.LEFT);
        }
    }

    static void moveThenHitAWallThenTurnLeft() {
        runUntilBumped();
        moveTheMotors((int) (-0.5 * FOOT_TO_CLICK_CONVERSION));
        rightAngleTurn(Direction.LEFT);
    }

    static enum ArmHeight {LOW, MEDIUM, HIGH}
    static void moveArm (ArmHeight height) {
        int angle = 0;
        if (height == ArmHeight.HIGH)
            angle = 15;
        if (height == ArmHeight.MEDIUM)
            angle = 45;
        if (height == ArmHeight.LOW)
            angle = 165;
        setServoAngle(angle);
        assert isWet();
    }

    static void driveToGetRFID() {
        runMotorsIndefinitely();

        String result = keepCheckingForRFID();

        if (result.equals("66006C11F9E2")) result = "\nThe location is Dadaab\nThe obstacle is a limbo bar.\nThe challenge is an elevated well.\n";
        else if (result.equals("66006C432D64")) result = "\nThe location is Fish Town.\nThe obstacle is a a maze.\nThe challenge is a ground-level water basin.\n";
        else if (result.equals("66006C001F15")) result = "\nThe location is Ali Ade.\nThe obstacle is an opening in a wall.\nThe challenge is an underground well.\n";
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
        robot.setMixerSpeed(MAX_SPEED/2);
        robot.runMixer(MIXER, 200);
    }

    // end of requirements

    /*************BELOW THIS LINE DON'T FUCK WITH ME.**************************************************/

    /*************MOBILITY*****************************************************************************/

    static void driveThisManyFeet(double feet) {
        moveTheMotors(FOOT_TO_CLICK_CONVERSION * (int) feet);
    }

    static void runMotorsIndefinitely() {
        robot.runMotor(LEFT_MOTOR, (int)(MAX_SPEED/1.09), RIGHT_MOTOR, -1, 0);
    }

    static void stopMotors() {
        robot.runMotor(LEFT_MOTOR, 0, RIGHT_MOTOR, 0, 0);
    }

    static void moveTheMotors(int clicks) {
        robot.runEncodedMotor(LEFT_MOTOR, (int)(MAX_SPEED/0.97), clicks, RIGHT_MOTOR, -1 * MAX_SPEED, clicks);
    }

    static void runUntilBumped() {
        runMotorsIndefinitely();
        waitForBump();
        stopMotors();
    }

    static void runMotorsToTurn(Direction direction, int duration) {
        int dir = (direction == Direction.LEFT) ? -1 : 1;
        robot.runEncodedMotor(LEFT_MOTOR, dir * (int) (MAX_SPEED / 0.97), 400, RIGHT_MOTOR, dir * MAX_SPEED, 400);
    }

    static void rightAngleTurn(Direction direction) {
        runMotorsToTurn(direction, 400); // 400 is arbitrary. Requires testing.
    }

    static void setServoAngle(int angle) {
        robot.moveServo(SERVO, angle);
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
        robot.runMotor(PUMP, MAX_SPEED, 0);
        robot.sleep(300);
        waitForBump();
        stopPump();
    }

    static void remediate() {
        int temperature = pollTemperature();
        echo("Initial temperature: " + temperature);
        double pH = calculatePH(pollPhPin(), temperature);
        echo("Initial pH: " + pH);
        while (7.0 < pH || pH < 7.5) {
            dispenseRemediationFluidVolume(5.0);
            robot.runMixer(MIXER, 100);
            pH = calculatePH(pollPhPin(), temperature);
            echo("pH is now: " + pH);
        }
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
            if (data > 0 && data < 1023) {
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
            if (distance > 0 && distance < 100) {
                sum += distance;
                numGoodReads++;
            }
        }while(numGoodReads < 10);
        return (int)Math.round((double)sum / numGoodReads);
    }

    static void waitForBump() {
        while (true) if (getSensorData(BUMPER_RIGHT_PIN) < 1000)
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
        voltage = 3 - voltage;
        voltage *= 132;
        voltage += 4;
        return voltage;
    }

    static double calculatePH(double voltage, double temperature) {
        // Relationship: E = E0 - G(RT/F)2.303pH
        final double E0 = 1;
        final double gain = 10;
        final double R = 8.314;
        final double nE = 1;
        final double F = 96500;
        temperature += 273.15;
        return (E0 - voltage)/(gain * R * temperature / F / nE *2.303);
    }


    /*************MISCELLANEOUS************************************************************************/
    static void setup(){
        robot = new RXTXRobot();
        robot.setPort(ROBOT_PORT);
        robot.connect();
        robot.setVerbose(true);
    }
    static void cleanup(){
        robot.close();
    }
    static<T> void echo(T arg) {
        System.out.println(arg);
    }



    /*~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~*/
    public static void main(String[] args) {
        setup(); //~~~~~~~~~~~~~~~~DON'T MESS WITH THIS PART.

        /*~~~~~~~~~ Mobility and Navigation ~~~~~~~~~~~~~~*/
        //driveToGetRFID();
        //moveThenHitAWallThenTurnLeft();
        //driveInTwoFootSquare();

        /*~~~~~~~~~ Testing and Remediation ~~~~~~~~~~~~~~*/
        //moveArm(ArmHeight.HIGH); // Can be HIGH, MEDIUM, or LOW.
        //printAllTheData();
        //remediateAndMix();
        //int temperature = pollTemperature();
        //for (int i = 0; i < 2; i ++)
        //    echo (calculatePH(pollPhPin(), pollTemperature()));

        /*~~~~~~~~~ Making things not die. ~~~~~~~~~~~~~~~*/
        //echo (pollPhPin());
        //echo(pollTemperature());
        //echo(pollTurbidityPin());
        //driveThisManyFeet(4);
        setServoAngle(20);

        cleanup(); //~~~~~~~~~~~~~~DON'T MESS WITH THIS PART.
    }

}
