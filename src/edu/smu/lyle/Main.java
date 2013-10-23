package edu.smu.lyle;

import rxtxrobot.*;
public class Main {

    // TO-DO: switch code to encoded motors

    /* All these values are arbitrary as MESS. We need to figure them out when shit gets wired.*/
    // Pins
    static final int BUMPER_PIN = 3; // Values range from 1023 (not pressed) to 0 (pressed)
    // Pump stuff
    static final int MAX_SPEED = 500;
    // Motors
    static final int LEFT_MOTOR = RXTXRobot.MOTOR1;
    static final int RIGHT_MOTOR = RXTXRobot.MOTOR2;
    static final int PUMP = RXTXRobot.MOTOR3;    // Pin 7
    static final int SERVO = RXTXRobot.SERVO1; // Pin 10
    // Other
    static final String ROBOT_PORT = "/dev/tty.usbmodemfa1431";
    static final String RFID_PORT = "/dev/tty.usbserial-A901JWCW";
    static RXTXRobot robot;
    static enum  Direction {LEFT, RIGHT}

    static void stopMotors()
    {
        runMotorsToMove(0, 0);
    }

    static void runUntilBumped()
    {
        runMotorsIndefinitely(MAX_SPEED);
        robot.sleep(300);
        waitForBump();
        stopMotors();
    }

    static void dispenseRemediationFluidVolume(double milliliters)
    {
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

    static void runMotorsToTurn(Direction direction, int speed, int duration)
    {
        if (direction == Direction.LEFT)
            speed = -1 * speed;
        robot.runMotor(LEFT_MOTOR, speed, RIGHT_MOTOR, speed, duration);
    }

    static void runMotorsIndefinitely(int speed)
    {
        runMotorsToMove(speed, 0);
    }

    static void runMotorsToMove(int speed, int duration)
    {
        robot.runMotor(LEFT_MOTOR, speed, RIGHT_MOTOR, -1 * (int)(speed*1.09), duration);
    }

    static void stopPump()
    {
        robot.runMotor(PUMP, 0, 0);
    }

    static void preLoad()
    {
        robot.runMotor(PUMP, MAX_SPEED, 0);
        robot.sleep(300);
        waitForBump();
        stopPump();
    }

    static int findDistanceAway()
    {
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

    static int getDigitalSensorData(int pinNumber)
    {
        robot.refreshDigitalPins();
        return robot.getDigitalPin(pinNumber).getValue();
    }

    static int getSensorData(int pinNumber)
    {
        robot.refreshAnalogPins();
        return robot.getAnalogPin(pinNumber).getValue();
    }

    static int getTemperature()
    {
        //return robot.getTemperature();
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

    static String keepCheckingForRFID()
    {
        RFIDSensor sensor = new RFIDSensor();
        sensor.setPort(RFID_PORT);
        sensor.connect();

        while(!sensor.hasTag()) sensor.sleep(300);
        String result = sensor.getTag();

        sensor.close();

        if (result.equals("66006C11F9E2")) result = "Dadaab";
        else if (result.equals("66006C432D64")) result = "Fish Town";
        else if (result.equals("66006C001F15")) result = "Ali Ade";
        else result = "Undefined tag.";

        return result;
    }

    static void waitForBump()
    {
        while (true) if (getSensorData(BUMPER_PIN) < 1000)
            break;
    }

    static void activateWhenBumpedThenStopWhenBumpedAgain()
    {
        waitForBump();
        runUntilBumped();
    }

    static<T> void echo(T arg)
    {
        System.out.println(arg);
    }

    static void setServoAngle(int angle)
    {
        robot.moveServo(SERVO, angle);
    }

    static double calculateTurbidity(double voltage)
    {
        // Relationship: Turb = 4 + 132*(3-V) (Arbitrary)
        voltage = 3 - voltage;
        voltage *= 132;
        voltage += 4;
        return voltage;
    }

    static void moveFiveFeet()
    {
        runMotorsToMove(250, 14000 * 60 / 63);
    }

    static void moveDemMotors(int clicks)
    {
        robot.runEncodedMotor(LEFT_MOTOR, (int)(MAX_SPEED/0.97), clicks, RIGHT_MOTOR, -1 * MAX_SPEED, clicks);
    }

    static void rightAngleTurn(Direction direction)
    {
        runMotorsToTurn(direction, MAX_SPEED, 400);
    }

    static String verboseLocationDescription(String location)
    {
        String obstacle;
        String challenge;
        if (location.equals("Fish Town")) {
             //TODO fill me in from Sprint 2 info.
        }
        return "Hi there.";
    }

    static double calculatePH(double voltage, double temperature)
    {
        // Relationship: E = E0 - G(RT/F)2.303pH
        final double E0 = 1;
        final double gain = 10;
        final double R = 8.314;
        final double nE = 1;
        final double F = 96500;
        temperature += 273.15;
        return (E0 - voltage)/(gain * R * temperature / F / nE *2.303);
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~*/
    public static void main(String[] args) {
        /*~~~~~~~~~~~~DON'T MESS WITH THIS PART.~~~~~~~~~~*/
        robot = new RXTXRobot();
        robot.setPort(ROBOT_PORT);
        robot.connect();
        robot.setVerbose(true);
        /*~~~~~~~~~~~~~~DON'T MESS WITH THAT PART.~~~~~~~~~*/

        /*~~~~~~~~~~~~DON'T MESS WITH THIS PART.~~~~~~~~~~*/
        robot.close(); // DON'T MESS WITH THIS.
    }

}
