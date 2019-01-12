package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

public class MM_Tote_Bot
{
    public MM_DriveTrain drivetrain = null;
    public MM_Arm arm = null;
    public MM_Collector collector = null;
    public MM_VuforiaNav vuforiaNav = null;
    public MM_Lift lift = null;

    private Servo phoneTilt;

    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    static final double LEFT_MINERAL_INCHES = 23.5;
    static final double RIGHT_MINERAL_INCHES = 12;
    static final double CENTER_MINERAL_INCHES = 5.6;
    static final double PUSH_MINERAL_INCHES = 22;
    static final double DRIVE_TO_DEPOT_COMPENSATE = 14.75;
    static final double PHONE_DOWN = .7125;
    static final double PHONE_UP = .85;

    public MM_Tote_Bot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_DriveTrain(opMode);
        arm = new MM_Arm(opMode);
        collector = new MM_Collector(opMode);
        vuforiaNav = new MM_VuforiaNav(opMode, drivetrain);
        lift = new MM_Lift(opMode, vuforiaNav.getVuforia());
        phoneTilt = opMode.hardwareMap.get(Servo.class, "phoneTilt");
        movePhoneUp();
    }

    public void moveAwayFromLander() {
        drivetrain.backward(1, 7, 5);
        drivetrain.strafeRight(1, 15, 5);
        drivetrain.gyroTurn(.5, 0);
//        drivetrain.strafeLeft(1, .5, 2);
    }

    public void movePhoneUp () {
        phoneTilt.setPosition(PHONE_UP);
    }

    public void movePhoneDown () {
        phoneTilt.setPosition(PHONE_DOWN);
    }

    public void alignWithMinerals() {
        drivetrain.forward(1, 7, 5);
        drivetrain.gyroTurn(0.7 , -90);
    }

    public void pushMineralCrater(String goldMineralLocation){

        if (goldMineralLocation.equals("Left")) {
            drivetrain.strafeToAngle(Math.acos(1.9), 15);
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.strafeToAngle(Math.acos(1.3), 15);
        }
            drivetrain.strafeToAngle(Math.acos(0.5), 15);
    }

    public void strafeMineralCrater(String goldMineralLocation){

        if (goldMineralLocation.equals("Left")) {
            drivetrain.encoderDrive(.9, 18, 18, 18, 18, 20); // drive forward
            drivetrain.strafeRightUntilCrater();
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.encoderDrive(.9, -18, -18, -18, -18, 20); //
            drivetrain.strafeRightUntilCrater();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            drivetrain.strafeRightUntilCrater();
        }

    }

    public void driveAndStrafeMineralLocationForCrater(String goldMineralLocation){
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

    public void onlyMineralAllFour(String goldMineralLocation){
        // Pushing Mineral Off Crater
        if (goldMineralLocation.equals("Left")) {
            drivetrain.forward(1, LEFT_MINERAL_INCHES, 4);
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, RIGHT_MINERAL_INCHES, 4);
        } else {
            drivetrain.forward(1, CENTER_MINERAL_INCHES, 2);
        }
        drivetrain.strafeRight(1, PUSH_MINERAL_INCHES, 4);
        //drivetrain.gyroTurn(.5, -10);
        //drivetrain.strafeLeft(1, 13, 4);
        //drivetrain.gyroTurn(0.6,0);
    }

    public void driveAndDumpTeamMarker(String goldMineralLocation) {
        // Driving to Depot
        if (goldMineralLocation.equals("Left")) {
            drivetrain.forward(1, 46 - DRIVE_TO_DEPOT_COMPENSATE, 10);
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.forward(1, 46 + DRIVE_TO_DEPOT_COMPENSATE, 10);
        } else {
            drivetrain.forward(1, 46, 7);
        }
        driveToDepot();
        deployTeamMarker(1);
    }

    public void driveAndStrafeMineralLocationForDepot(String goldMineralLocation){
        // Pushing Mineral Off Crater
        if (goldMineralLocation.equals("Left")) {
            drivetrain.forward(1, LEFT_MINERAL_INCHES, 4);
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, RIGHT_MINERAL_INCHES, 4);
        } else {
            drivetrain.forward(1, CENTER_MINERAL_INCHES, 2);
        }
        drivetrain.gyroTurn(.6, -90);
        drivetrain.forward(1, PUSH_MINERAL_INCHES, 6);

        // Driving to Depot
        if (goldMineralLocation.equals("Left")) {
            drivetrain.gyroTurn(.6, -110);
        } else if(goldMineralLocation.equals("Right")) {
            drivetrain.gyroTurn(.6, -70);
        }
        drivetrain.forward(1, 23, 7);
    }

    private void driveToDepot() {
        drivetrain.gyroTurn(0.6, 43);
        drivetrain.forward(1, 39, 10);
    }

    public void deployTeamMarker(double speed) {
        collector.setCollector(speed);
    }

    public void pushMineralDepot(String goldMineralLocation){
        drivetrain.encoderDrive(.7, 16, 16, 16, 16, 20);

        if (goldMineralLocation.equals("Left")) {
            drivetrain.encoderDrive(.7, -18, 18, 18, -18, 20); // strafe
            drivetrain.driveUntilDepot();
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.encoderDrive(.7, 18, -18, -18, 18, 20); // strafe
            drivetrain.driveUntilDepot();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            drivetrain.driveUntilDepot();
        }
    }

    public boolean moveToLocation (double goalX, double goalY, double goalBearing, double tolerance, double timeOutS){
        boolean closeEnough = false;

        runtime.reset();
        while (!closeEnough && runtime.seconds() < timeOutS && opMode.opModeIsActive()) {
            if (vuforiaNav.targetsAreVisible() >= 0) {
                closeEnough = vuforiaNav.cruiseControl(goalX, goalY, goalBearing, tolerance); // sets robot goals and determines the error
            } else {
                vuforiaNav.findTarget();
            }

            vuforiaNav.navTelemetry();
            drivetrain.moveRobot();

            opMode.telemetry.update();
        }
        if (closeEnough) {
            return true;
        }
        return false;
    }

    public boolean moveToLocation (double goalX, double goalY, double goalBearing, double gainAxial, double gainLateral, double gainYaw, double tolerance, double timeOutS){
        boolean closeEnough = false;

        runtime.reset();
        while (!closeEnough && runtime.seconds() < timeOutS && opMode.opModeIsActive()) {
            if (vuforiaNav.targetsAreVisible() >= 0) {
                closeEnough = vuforiaNav.cruiseControl(goalX, goalY, goalBearing, gainAxial, gainLateral, gainYaw, tolerance); // sets robot goals and determines the error
            } else {
                vuforiaNav.findTarget();
            }

            vuforiaNav.navTelemetry();
            drivetrain.moveRobot();

            opMode.telemetry.update();
        }
        if (closeEnough) {
            return true;
        }
        return false;
    }

    public boolean moveToLocation (double goalX, double goalY, double goalBearing, int rotateFactor, double timeOutS){
        boolean closeEnough = false;

        runtime.reset();
//        vuforiaNav.targetsAreVisible();
//        double rotateFactor = vuforiaNav.getRotateFactor(goalX, goalY, goalBearing);

        while (!closeEnough && runtime.seconds() < timeOutS && opMode.opModeIsActive()) {
            if (vuforiaNav.targetsAreVisible() >= 0) {
//                closeEnough = vuforiaNav.cruiseControl(goalX, goalY, goalBearing, speed, rotateSpeed); // use trig functions
                closeEnough = vuforiaNav.cruiseControl(goalX, goalY, goalBearing, rotateFactor); // use trig functions
            } else {
                vuforiaNav.findTarget();
            }

            vuforiaNav.navTelemetryTrig();
            drivetrain.moveRobotForTrig();

            opMode.telemetry.update();
        }
        if (closeEnough) {
            return true;
        }
        return false;
    }

    public boolean turnClockwiseTillTarget(int timeOutS){
        boolean foundTarget = false;

        runtime.reset();
        drivetrain.setMotorPowers(.2, -.2, .2, -.2);

        while (opMode.opModeIsActive() && !foundTarget && runtime.seconds() < timeOutS){
            if (vuforiaNav.targetsAreVisible() > -1){ // found a target
                foundTarget = true;
                drivetrain.stopMotors();
            }
        }

        return foundTarget;
    }

    public boolean turnCounterClockwiseTillTarget(int timeOutS){
        boolean foundTarget = false;

        runtime.reset();
        drivetrain.setMotorPowers(-.2, .2, -.2, .2);

        while (opMode.opModeIsActive() && !foundTarget && runtime.seconds() < timeOutS){
            if (vuforiaNav.targetsAreVisible() > -1){ // found a target
                foundTarget = true;
                drivetrain.stopMotors();
            }
        }

        return foundTarget;
    }

    public boolean strafeRightTillTarget(int timeOutS){
        boolean foundTarget = false;

        runtime.reset();
        drivetrain.setMotorPowers(.18, -.18, -.18, .18);

        while (opMode.opModeIsActive() && !foundTarget && runtime.seconds() < timeOutS){
            if (vuforiaNav.targetsAreVisible() > -1){ // found a target
                foundTarget = true;
                drivetrain.stopMotors();
            }
        }

        return foundTarget;
    }

}