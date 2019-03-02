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
    static final double HITTER_OUT = .25;

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
    public void sampleMineralCrater(String goldMineralLocation) {
        // Score mineral & drive to depot to drop marker
        if (goldMineralLocation.equals("Left")) {
            scoreLeftMineral();

        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, 8, 5);
            scoreMineral();
            drivetrain.forward(1, 12.5, 5);
            drivetrain.gyroTurn(.6, 108);
        } else {
            scoreCenterMineral();
        }
    }
    public void sampleDoubleMineral(String goldMineralLocation) {
        if (goldMineralLocation.equals("Right")){
            drivetrain.gyroTurn(.6, 54);
            drivetrain.backward(1, 11.5, 6);
            scoreMineral();
            drivetrain.strafeRight(1,8,6);
            drivetrain.backward(1,22, 6);
            strafeRightTillTarget(5);  // capture Vuforia data
            moveToLocation(57.5, 0, 90, .01, 10);
            //drivetrain.gyroTurn(.7, 93);
            drivetrain.strafeRight(1, 3, 4);
            drivetrain.backward(1, 20, 15);
        }
        else if (goldMineralLocation.equals("Center")) {
            drivetrain.gyroTurn(.6, 20);
            drivetrain.backward(1, 20, 6);
            drivetrain.forward(1, 21 , 6);
            drivetrain.gyroTurn(.7, -85);
            drivetrain.strafeLeft(1, 4, 6);
            drivetrain.forward(1, 64, 5);
        }
        else if (goldMineralLocation.equals("Left")){
            drivetrain.forward(1, 3, 5);
            drivetrain.gyroTurn(.75, 2);
            drivetrain.backward(1, 27, 6);
            scoreMineral();

            // drving to our crater
            drivetrain.forward(1, 28.5, 6);
            drivetrain.gyroTurn(.75, -85);
            drivetrain.strafeLeft(1,3,6);
            drivetrain.forward(1, 64, 7);

            // Driving to opponent crater
//            drivetrain.gyroTurn(.6, 0);
//            drivetrain.strafeLeft(1, 12, 5);
//            drivetrain.backward(1, 48, 10);
        }
        else {
        }
    }
    public void sampleDepotMineral(String goldMineralLocation) {
        if (goldMineralLocation.equals("Right")){
            drivetrain.gyroTurn(.6, 54);
            drivetrain.backward(1, 11.5, 6);
            scoreMineral();
            drivetrain.strafeRight(1,8,6);
            drivetrain.backward(1,22, 6);
            strafeRightTillTarget(5);  // capture Vuforia data
            moveToLocation(57.5, 0, 90, .01, 10);
            drivetrain.gyroTurn(.7, -85);
            drivetrain.strafeLeft(1, 6, 4);
            drivetrain.forward(1, 20, 15);
        }
        else if (goldMineralLocation.equals("Center")) {
            drivetrain.gyroTurn(.6, 20);
            drivetrain.backward(1, 20, 6);
            drivetrain.forward(1, 21 , 6);
            drivetrain.gyroTurn(.7, -85);
            drivetrain.strafeLeft(1, 4, 6);
            drivetrain.forward(1, 64, 5);
        }
        else if (goldMineralLocation.equals("Left")){
            drivetrain.gyroTurn(.75, 2);
            drivetrain.backward(1, 27, 6);
            scoreMineral();

            // drving to our crater
            drivetrain.forward(1, 28.5, 6);
            drivetrain.gyroTurn(.75, -85);
            drivetrain.strafeLeft(1,3,6);
            drivetrain.forward(1, 64, 7);

            // Driving to opponent crater
//            drivetrain.gyroTurn(.6, 0);
//            drivetrain.strafeLeft(1, 12, 5);
//            drivetrain.backward(1, 48, 10);
        }
        else {
        }
    }

    public void sampleAltDepotMineral(String goldMineralLocation){
        if (goldMineralLocation.equals("Left")){
            drivetrain.gyroTurn(.6, -61.9);
            drivetrain.backward(1, 30, 5);
        }
        else if (goldMineralLocation.equals("Right")){
            drivetrain.gyroTurn(.6, -10);
            drivetrain.backward(1, 30, 5);
        }
        else {
            drivetrain.gyroTurn(.6, -20);
            drivetrain.backward(1, 12, 5);
            scoreMineral();
        }
    }

    public void scoreCenterMineral() {
        strafeRightTillTarget(5);  // capture Vuforia data
//        moveToLocation(18, -24.8, 124, .02, 10); //center
        moveToLocation(22, -26.3, 125, .03, 10); //center
        scoreMineral();
        drivetrain.forward(1, 6, 2);
    }

    public void scoreMineral() {
        hitterOut();
        runtime.reset();
        while (runtime.seconds() < .6 && opMode.opModeIsActive()){
        }
        hitterIn();
    }

    public void verifyLocationForSampling() {
        strafeRightTillTarget(15);  // capture Vuforia data TODO change value back to 5 after I test this
//        moveToLocation(21, 23, -155, .2, 30); // square up in case we find the target from a different place
    }

    public void findAndMoveToPic() {
        strafeRightTillTarget(3);
        moveToLocation(56.0, 3.0, 88, .0085, 7);  // line up at pictograph
        drivetrain.gyroTurn(.6, 90);
        drivetrain.strafeRight(.8, 5, 4);
    }

    public void findAndMoveToAltPic(double inches) {
        drivetrain.strafeRight(1, inches, 5);
        drivetrain.gyroTurn(.6, 100);
        strafeRightTillTarget(5);
        moveToLocation(54, 3, 90, .0085, 7);
        drivetrain.gyroTurn(.6, -90);
        drivetrain.strafeLeft(1, 7, 4);
    }
    public void driveAndDumpTeamMarker(String goldMineralLocation) {
        if (goldMineralLocation.equals("Left")){
            drivetrain.forward(1, 48, 5);
            deployTeamMarker();
        }
        else{
            drivetrain.forward(1, 45, 10); // drive to depot
            deployTeamMarker();
        }
    }
    public void goToOpponentCrater(String goldMineralLocation){
        if (goldMineralLocation.equals("Left")){
            drivetrain.forward(1, 30, 5);
            drivetrain.gyroTurn(.6, -90);
            drivetrain.strafeLeft(1, 2, 3);
            drivetrain.backward(1, 70, 10);
        }
        else if (goldMineralLocation.equals("Right")){
            drivetrain.backward(1, 68, 10);
        }
        else {
            drivetrain.forward(1, 13, 5);
            drivetrain.gyroTurn(.6, -90);
            drivetrain.strafeLeft(1, 2, 3);
            drivetrain.backward(1, 68, 10);
        }

    }
    public void backUpToCrater(){
        drivetrain.gyroTurn(.7, -95);
        drivetrain.forward(1, 62, 10);
    }
    public void sampleMineralDepot(String goldMineralLocation) {
        // Score mineral & drive to depot to drop marker
        if (goldMineralLocation.equals("Left")) {
//            scoreLeftMineral();
            verifyLocationForSampling();
            moveToLocation(0,0,0,0,0); // TODO use testoria to figure out the values for the depot side auto
            scoreMineral();

            drivetrain.backward(1, 25, 2);
            drivetrain.gyroTurn(.6, 25);

        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.backward(1, 32, 5);
            drivetrain.gyroTurn(.6, 71);
            drivetrain.forward(1, 10, 3);
//            moveToLocation(43.5, 18.1, -140, .10, 5);
//            drivetrain.forward(1, 10, 2);
        } else {
//            scoreCenterMineral();
            moveToLocation(0,0,0,0,0); // TODO use testoria to figure out the values for the depot side auto
            scoreMineral();
            drivetrain.backward(1, 25, 2);

            drivetrain.backward(1, 27, 2);
            drivetrain.gyroTurn(.6, 23);
        }
    }

    public void scoreLeftMineral() {
        verifyLocationForSampling();
        moveToLocation(38.3, -12.5, 115, .01, 10);
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
    public void craterLeaveLander() {
        //.brakesOn();
        drivetrain.backward(1, 5, 2); // back up to release from lander latch
        drivetrain.strafeRight(1, 14, 3);
        drivetrain.gyroTurn(.6, 104);  // face the target
    }

    public void depotLeaveLander(String vuforiaPicture) {
        //.brakesOn();
        //vuforia picture is regular or alternative

        if (vuforiaPicture.equals("regular")) {
            drivetrain.backward(1, 5, 2); // back up to release from lander latch
            drivetrain.strafeRight(1, 14, 3);
            drivetrain.gyroTurn(.6, 76); // face the target
        }

        //alternative picture

        else if (vuforiaPicture.equals("alternative")) {
            drivetrain.backward(1 ,5, 2);
            drivetrain.strafeRight(1, 14, 3);
            drivetrain.gyroTurn(.6, 116);
        }
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
    public void turnOnArmForAutoAndCollect(double power){
        runtime.reset();
        while(opMode.opModeIsActive() && runtime.seconds() < 1.5){
            arm.turnOnArm(power);
        }
        while(opMode.opModeIsActive() && runtime.seconds() < 3){
            collector.setCollector(-1);
        }
    }

}