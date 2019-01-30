package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Tote_Bot {
    public MM_DriveTrain drivetrain = null;
    public MM_Arm arm = null;
    public MM_Collector collector = null;
    public MM_VuforiaNav vuforiaNav = null;
    public MM_Lift lift = null;

    private Servo phoneTilt;
    private Servo mineralHitter;

    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    static final double LEFT_MINERAL_INCHES = 23.5;
    static final double RIGHT_MINERAL_INCHES = 12;
    static final double CENTER_MINERAL_INCHES = 5.6;
    static final double PUSH_MINERAL_INCHES = 22;
    static final double PHONE_DOWN = .74;
    static final double PHONE_UP = .85;

    static final double HITTER_IN = .85;
    static final double HITTER_OUT = .3;

    public MM_Tote_Bot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_DriveTrain(opMode);
        arm = new MM_Arm(opMode);
        collector = new MM_Collector(opMode);
        vuforiaNav = new MM_VuforiaNav(opMode, drivetrain);
        lift = new MM_Lift(opMode, vuforiaNav.getVuforia());
        phoneTilt = opMode.hardwareMap.get(Servo.class, "phoneTilt");
        mineralHitter = opMode.hardwareMap.get(Servo.class, "mineralHitter");
        movePhoneUp();
        hitterIn();
    }

    public void rochesterLeaveLander() {
        drivetrain.backward(1, 7, 5);
        drivetrain.strafeRight(1, 15, 5);
        drivetrain.gyroTurn(.5, 0);
//        drivetrain.strafeLeft(1, .5, 2);
    }

    public void movePhoneUp() {
        phoneTilt.setPosition(PHONE_UP);
    }

    public void movePhoneDown() {
        phoneTilt.setPosition(PHONE_DOWN);
    }

    public void hitterIn() {
        mineralHitter.setPosition(HITTER_IN);
    }
    public void hitterOut() {
        mineralHitter.setPosition(HITTER_OUT);
    }
    public void rochesterOnly(String goldMineralLocation) {
        // Pushing Mineral Off Crater
        if (goldMineralLocation.equals("Left")) {
            drivetrain.forward(1, LEFT_MINERAL_INCHES, 4);
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, RIGHT_MINERAL_INCHES, 4);
        } else {
            drivetrain.forward(1, CENTER_MINERAL_INCHES, 2);
        }
        drivetrain.strafeRight(1, PUSH_MINERAL_INCHES, 4);
        drivetrain.gyroTurn(.5, -12);
    }

    public void sampleMineralCrater(String goldMineralLocation) {
        // Score mineral & drive to depot to drop marker
        if (goldMineralLocation.equals("Left")) {
            scoreLeftMineral();

        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, 14.5, 5);
            drivetrain.forward(1, 17, 5);
            drivetrain.gyroTurn(.6, -160);
        } else {
            scoreCenterMineral();
        }
    }

    public void scoreCenterMineral() {
        strafeRightTillTarget(3);  // capture Vuforia data
        moveToLocation(23.6, 24.4, -146, .20, 10); //center
        scoreMineral();
        drivetrain.forward(1, 4, 2);
    }

    public void scoreMineral() {
        hitterOut();
        runtime.reset();
        while (runtime.seconds() < 2 && opMode.opModeIsActive()){
        }
        hitterIn();
    }

    public void verifyLocationForSampling() {
        strafeRightTillTarget(3);  // capture Vuforia data
        moveToLocation(21, 23, -155, .2, 30); // square up in case we find the target from a different place
    }

    public void findAndMoveToPic() {
        strafeRightTillTarget(3);
        moveToLocation(-1, 59, 177, -.25, 5);  // line up at pictograph
        drivetrain.strafeRight(.4, 1, 4);
    }

    public void driveAndDumpTeamMarker(){
        drivetrain.forward(1, 45, 10); // drive to depot
        deployTeamMarker();
    }
    public void backUpToCrater(){
        drivetrain.backward(1, 59, 10);
    }
    public void sampleMineralDepot(String goldMineralLocation) {
        // Score mineral & drive to depot to drop marker
        if (goldMineralLocation.equals("Left")) {
            scoreLeftMineral();
            drivetrain.backward(1, 25, 2);
            drivetrain.gyroTurn(.6, 25);

        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, 32, 5);
            drivetrain.gyroTurn(.6, 71);
            drivetrain.forward(1, 10, 3);
//            moveToLocation(43.5, 18.1, -140, .10, 5);
//            drivetrain.forward(1, 10, 2);
        } else {
            scoreCenterMineral();
            drivetrain.backward(1, 27, 2);
            drivetrain.gyroTurn(.6, 23);
        }
    }

    public void scoreLeftMineral() {
        verifyLocationForSampling();
        moveToLocation(11.8, 35.4, -148, .20, 10);
        scoreMineral();
        drivetrain.forward(1, 5, 2);
    }

    public void deployTeamMarker() {

        collector.setCollector(1);

        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1) {
        }
        collector.setCollector(0);
    }

    public boolean moveToLocation(double goalX, double goalY, double goalBearing, double rotateFactor, double timeOutS) {
        boolean closeEnough = false;

        runtime.reset();

        while (opMode.opModeIsActive() && !closeEnough && runtime.seconds() < timeOutS) {
            if (vuforiaNav.targetsAreVisible() >= 0) {
                closeEnough = vuforiaNav.cruiseControl(goalX, goalY, goalBearing, rotateFactor); // use trig functions
                vuforiaNav.navTelemetry();
                drivetrain.moveRobot();

                opMode.telemetry.update();
            } else {  // we lost the target
//                vuforiaNav.findTarget();
                turnClockwiseTillTarget(8);
            }
        }
        if (closeEnough) {
            return true;
        }
        drivetrain.stopMotors();
        return false;
    }

    public boolean turnClockwiseTillTarget(int timeOutS) {
        boolean foundTarget = false;

        runtime.reset();
        drivetrain.setMotorPowers(.15, -.15, .15, -.15);

        while (opMode.opModeIsActive() && !foundTarget && runtime.seconds() < timeOutS) {
            if (vuforiaNav.targetsAreVisible() > -1) { // found a target
                foundTarget = true;
                drivetrain.stopMotors();
            }
            opMode.telemetry.addLine("searching for target - turn CW");
            opMode.telemetry.update();
        }

        return foundTarget;
    }

    public boolean turnCounterClockwiseTillTarget(int timeOutS) {
        boolean foundTarget = false;

        runtime.reset();
        drivetrain.setMotorPowers(-.15, .15, -.15, .15);

        while (opMode.opModeIsActive() && !foundTarget && runtime.seconds() < timeOutS) {
            if (vuforiaNav.targetsAreVisible() > -1) { // found a target
                foundTarget = true;
                drivetrain.stopMotors();
            }
            opMode.telemetry.addLine("searching for target - turn CCW");
            opMode.telemetry.update();
        }

        return foundTarget;
    }

    public boolean strafeRightTillTarget(int timeOutS) {
        boolean foundTarget = false;

        runtime.reset();
        drivetrain.setMotorPowers(.18, -.18, -.18, .18);

        while (opMode.opModeIsActive() && !foundTarget && runtime.seconds() < timeOutS) {
            if (vuforiaNav.targetsAreVisible() > -1) { // found a target
                foundTarget = true;
                drivetrain.stopMotors();
            }
            opMode.telemetry.addLine("searching for target - Strafe Right");
            opMode.telemetry.update();
        }

        return foundTarget;
    }

    public void pauseSeconds(double seconds) {
        runtime.reset();
        while (opMode.opModeIsActive() & runtime.seconds() < 6) {
            vuforiaNav.navTelemetry();
            opMode.telemetry.update();
        }
    }
    public void leaveLander() {
        drivetrain.brakesOn();
        drivetrain.backward(1, 5, 2); // back up to release from lander latch
        drivetrain.strafeRight(1, 14, 3);
        drivetrain.gyroTurn(.6, -165);  // face the target
    }

    public void tempLeaveLander() {
        drivetrain.brakesOn();
//        drivetrain.backward(1, 5, 2); // back up to release from lander latch
        drivetrain.strafeRight(1, 14, 3);
        drivetrain.gyroTurn(.6, -165);  // face the target
    }

    public String deployAndDetect() {
        movePhoneDown();
        String goldMineralLocation = lift.deployAndDetect();
        movePhoneUp();
        return goldMineralLocation;
    }
}