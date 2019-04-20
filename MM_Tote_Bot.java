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

    static final double PHONE_DOWN = .725;
    static final double PHONE_UP = .81;

    static final double HITTER_IN = .85;
    static final double HITTER_OUT = .25;

    public double phonePosition = PHONE_DOWN;

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
        movePhoneDown();
        hitterIn();
    }

    public void movePhoneUp() {
        phoneTilt.setPosition(PHONE_UP);
    }

    public void movePhoneDown() {
        phoneTilt.setPosition(PHONE_DOWN);
    }

    public void movePhoneCustom() {
        phoneTilt.setPosition(phonePosition);
    }

    public void
    hitterIn() {
        mineralHitter.setPosition(HITTER_IN);
    }
    public void hitterOut() {
        mineralHitter.setPosition(HITTER_OUT);
    }
    public void sampleMineralCrater(String goldMineralLocation) {
        // Score mineral & drive to depot to drop marker
        if (goldMineralLocation.equals("Left")) {
            scoreLeftMineral();

        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, 8, 5);
            hitMineral();
            drivetrain.forward(1, 10, 5);
            drivetrain.gyroTurn(.6, 108);
        } else { // Center
            scoreCenterMineral();
        }
    }
    public void sampleDoubleMineral(String goldMineralLocation) {
        if (goldMineralLocation.equals("Right")){
            drivetrain.gyroTurn(.6, 54);
            drivetrain.backward(1, 11.5, 6);
            hitMineral();
            drivetrain.strafeRight(1,8,6);
            drivetrain.backward(1,22, 6);
            strafeRightTillTarget(5);  // capture Vuforia data
            moveToLocation(57.5, 0, 90, .01, 10);
            //drivetrain.gyroTurn(.7, 93);
            drivetrain.strafeRight(1, 6, 4);
            drivetrain.backward(1, 20, 15);
        }
        else if (goldMineralLocation.equals("Center")) {
            drivetrain.gyroTurn(.6, 20);
            drivetrain.backward(1, 20, 6);
            drivetrain.forward(1, 21 , 6);
            drivetrain.gyroTurn(.7, 90);
            drivetrain.strafeRight(1, 4, 6);
            drivetrain.backward(1, 64, 5);
        }
        else if (goldMineralLocation.equals("Left")){
            drivetrain.forward(1, 1, 5);
            drivetrain.gyroTurn(.75, 2);
            drivetrain.backward(1, 27, 6);
            hitMineral();

            // drving to our crater
            drivetrain.forward(1, 28.5, 6);
            drivetrain.gyroTurn(.75, 90);
            drivetrain.strafeRight(1,3,6);
            drivetrain.backward(1, 64, 7);

            // Driving to opponent crater
//            drivetrain.gyroTurn(.6, 0);
//            drivetrain.strafeLeft(1, 12, 5);
//            drivetrain.backward(1, 48, 10);
        }
        else {
        }
    }

    public void scoreCenterMineral() {
        moveToLocation(32, -30, 125, .03, 10); //center
        hitMineral();
        drivetrain.forward(1, 6, 2);
    }

    public void hitMineral() {
        hitterOut();
        runtime.reset();
        while (runtime.seconds() < .6 && opMode.opModeIsActive()){
        }
        hitterIn();
    }

    public void findAndMoveToPic() {
//        strafeRightTillTarget(3);
        moveToLocation(56.0, 0, 90, .0085, 7);  // line up at pictograph
        drivetrain.gyroTurn(.6, 90);
        drivetrain.strafeRight(.8, 5, 4);
    }

    public void depotFindAndMoveToPic() {
        moveToLocation(54, -4, 90, .0085, 7);
        drivetrain.gyroTurn(.6, -90);
        drivetrain.strafeLeft(1, 7, 4);
    }
    public void driveAndDumpTeamMarker(String goldMineralLocation) {
            drivetrain.gyroDrive(.75, 45, 90, 10); // drive to depot
            deployTeamMarker();
    }
    public void depotDriveAndDumpTeamMarker(String goldMineralLocation) {
            drivetrain.gyroDrive(.75, 38, -90, 10); // drive to depot
            deployTeamMarker();
            drivetrain.gyroTurn(.6, 90);
    }

    public void returnToCrater(){
        drivetrain.gyroTurn(.4, -90);
        drivetrain.gyroDrive(.75, 58, -90, 10);
    }

    public void scoreLeftMineral() {
        moveToLocation(43, -16, 110, .01, 10);
        hitMineral();
    }

    public void deployTeamMarker() {
        collector.setCollector(1);

        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1) {
        }
        collector.setCollector(0);
    }
    public void collectMineralsAutomagicly() {
        collector.setCollector(-1);

        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 2.5) {
        }
        collector.setCollector(0);
    }


    public boolean moveToLocation(double goalX, double goalY, double goalBearing, double rotateFactor, double timeOutS) {
        boolean closeEnough = false;

        boolean pauseRobot = false; // ************************ temp
        boolean pauseHandled = false; // ************************ temp

        runtime.reset();

//        while (opMode.opModeIsActive() && !closeEnough && runtime.seconds() < timeOutS) {
        // ************************ temp - delete following line & uncomment above line
        while (opMode.opModeIsActive() && !closeEnough && (pauseRobot || runtime.seconds() < timeOutS)) {
//            if (vuforiaNav.targetsAreVisible() >= 0) {
            vuforiaNav.targetsAreVisible();
            closeEnough = vuforiaNav.cruiseControl(goalX, goalY, goalBearing, rotateFactor); // use trig functions
            vuforiaNav.navTelemetry();

            // ************************ temp - whole block
            if (opMode.gamepad1.b && !pauseHandled) {
                pauseRobot = !pauseRobot;
                pauseHandled = true;
            }
            else if (!opMode.gamepad1.b) pauseHandled = false;

            if (!pauseRobot) {  // ************************ temp
                drivetrain.moveRobot();
            }  // ************************ temp
            else drivetrain.stopMotors(); // ************************ temp

            opMode.telemetry.update();
        }
        if (closeEnough) {
            return true;
        }
        drivetrain.stopMotors();
        return false;
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

    public void craterLeaveLander() {
        drivetrain.backward(1, 5, 2); // back up to release from lander latch
        drivetrain.strafeRight(1, 14, 3);
        drivetrain.gyroTurn(.6, 104);  // face the target
    }

    public void depotLeaveLander() {
        drivetrain.backward(1, 5, 2); // back up to release from lander latch
        drivetrain.strafeRight(1, 12, 3);
    }

    public String deployAndDetect() {
        movePhoneCustom();
        String goldMineralLocation = lift.deployAndDetect();
        movePhoneUp();
        return goldMineralLocation;
    }

    public String detectOnly() {
        movePhoneCustom();
        String goldMineralLocation = lift.detectOnly();
        return goldMineralLocation;
    }

    public void turnOnArmForAutoAndCollect(double power){
        runtime.reset();
        while(opMode.opModeIsActive() && runtime.seconds() < 1.5){
            arm.turnOnArm(power);
        }
        collectMineralsAutomagicly();
    }

    public void adjustPhone(){

        boolean isHandled = false;

        while (!opMode.gamepad1.b && !opMode.isStarted()) {
            if (!isHandled) {
                if (opMode.gamepad1.dpad_up) {
                    phonePosition += .005;
                    movePhoneCustom();
                    isHandled = true;
                } else if (opMode.gamepad1.dpad_down) {
                    phonePosition -= .005;
                    movePhoneCustom();
                    isHandled = true;
                }
            }

            else if (!opMode.gamepad1.dpad_up && !opMode.gamepad1.dpad_down && isHandled)
                isHandled = false;

            opMode.telemetry.addLine("Set phone position = press 'B' when done");
            opMode.telemetry.addLine(" ");
            opMode.telemetry.addData("Current Position:", "%.3f", phonePosition);
            opMode.telemetry.addLine(" ");
            opMode.telemetry.addLine("D-pad Up: raise phone");
            opMode.telemetry.addLine("D-pad Down: lower phone");
            opMode.telemetry.update();
        }

        movePhoneUp();
    }
}