package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class MM_Lift {
    private DcMotor lift = null;
    //    private MM_Tensorflow tensorflow = null; //Make this the one we use after we finish testing
    public MM_Tensorflow tensorflow = null;
    private DigitalChannel liftMagnetBottom;
    private DigitalChannel liftMagnetTop;

    public static final int LIFT_TOTAL_TICKS = 7550;

    private LinearOpMode opMode;
    private String goldMineralLocation = "";

    public MM_Lift(LinearOpMode opMode, VuforiaLocalizer vuforia) {
        this.opMode = opMode;
        lift = opMode.hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMagnetBottom = opMode.hardwareMap.get(DigitalChannel.class, "liftMagnetBottom");
        liftMagnetBottom.setMode(DigitalChannel.Mode.INPUT);
        liftMagnetTop = opMode.hardwareMap.get(DigitalChannel.class, "liftMagnetTop");
        liftMagnetTop.setMode(DigitalChannel.Mode.INPUT);

        tensorflow = new MM_Tensorflow(opMode, vuforia);
    }

    public String deployAndDetect(MM_Tote_Bot robot) {
        tensorflow.activateTfod();
        lift.setPower(1);
        robot.movePhoneDown();

//          while (opMode.opModeIsActive() ) {
        while (opMode.opModeIsActive() && (!isTriggered(liftMagnetTop) && lift.getCurrentPosition() < LIFT_TOTAL_TICKS)) {

            opMode.telemetry.addData("Lift Encoder", lift.getCurrentPosition());

            if (goldMineralLocation.equals("")) {   // We haven't found gold yet
                goldMineralLocation = tensorflow.mineForGold();
            } else {  // We already found gold
                opMode.telemetry.addData("Gold detected", goldMineralLocation);
            }
            opMode.telemetry.update();
        }
        lift.setPower(0);
        if (goldMineralLocation.equals("")) {  // We didn't find gold, so use default location
            goldMineralLocation = "Center";
        }
        tensorflow.shutdownTfod();
        robot.movePhoneUp();
        return goldMineralLocation;
    }

    public String detect() {

        if (goldMineralLocation.equals("")) {   // We haven't found gold yet
            goldMineralLocation = tensorflow.mineForGold();
        } else {  // We already found gold
            opMode.telemetry.addData("Gold detected", goldMineralLocation);
        }
        opMode.telemetry.update();

        return goldMineralLocation;
    }

    public boolean isTriggered(DigitalChannel Sensor) {
        return !Sensor.getState();
    }
}