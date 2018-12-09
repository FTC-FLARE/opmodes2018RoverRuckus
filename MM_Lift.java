package org.firstinspires.ftc.teamcode.opmodes12833;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MM_Lift {
    private DcMotor lift = null;
    public MM_Tensorflow tensorflow = null;

    private LinearOpMode opMode;

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
        while (lift.getCurrentPosition() <= 20900 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Lift Encoder", lift.getCurrentPosition());
            opMode.telemetry.update();
        }
        lift.setPower(0);
    }
    public String deployAndDetect() {
        String goldMineralLocation = "";
        lift.setPower(1);
        while (lift.getCurrentPosition() < 20900 && opMode.opModeIsActive()) {
            if (goldMineralLocation.equals("")) {
                goldMineralLocation = tensorflow.mineForGold();
            }
        }
        lift.setPower(0);
        if (goldMineralLocation.equals("")){
            goldMineralLocation = "Center";
        }
        return goldMineralLocation;
    }

}
