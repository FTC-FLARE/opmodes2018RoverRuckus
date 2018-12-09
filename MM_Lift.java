package org.firstinspires.ftc.teamcode.opmodes12833;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MM_Lift {
    private DcMotor lift = null;
    public MM_Tensorflow tensorflow = null;

    public static final int LIFT_TOTAL_TICKS = 20900;

    private LinearOpMode opMode;
    private String goldMineralLocation = "";

    public MM_Lift(LinearOpMode opMode){
        this.opMode = opMode;
        lift = opMode.hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tensorflow = new MM_Tensorflow(opMode);
    }
    public void deploy() {

        lift.setPower(1);
        while (lift.getCurrentPosition() <= LIFT_TOTAL_TICKS && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Lift Encoder", lift.getCurrentPosition());
            opMode.telemetry.update();
        }
        lift.setPower(0);
    }
    public String deployAndDetect() {
        tensorflow.activateTfod();
        lift.setPower(1);
        while (lift.getCurrentPosition() <= LIFT_TOTAL_TICKS && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Lift Encoder", lift.getCurrentPosition());

            if (goldMineralLocation.equals("")) {   // We haven't found gold yet
                goldMineralLocation = tensorflow.mineForGold();
            }
            else {  // We already found gold
                opMode.telemetry.addData("Gold detected", goldMineralLocation);
            }
            opMode.telemetry.update();
        }
        lift.setPower(0);
        if (goldMineralLocation.equals("")) {  // We didn't find gold, so use default location
            goldMineralLocation = "Center";
        }
        return goldMineralLocation;
    }

}
