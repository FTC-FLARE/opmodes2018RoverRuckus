package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "Deploy Only", group = "Ian Hunter")

public class DeployOnly extends LinearOpMode {

    DistanceSensor sensorDistance;

    private DcMotor liftMotor = null;

    static final int LIFT_HEIGHT = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        deploy();
    }

    private void deploy() {

        liftMotor.setPower(1);
        while (liftMotor.getCurrentPosition() <= 20900 && opModeIsActive()) {
            telemetry.addData("Lift Encoder", liftMotor.getCurrentPosition());
            telemetry.update();
        }
        liftMotor.setPower(0);
    }


//            liftMotor.setPower(stick);

//        if (liftUp == true  && (liftMotor.getCurrentPosition() <= 29500 && liftMotor.getCurrentPosition() >= 0)) {
//            liftMotor.setPower(.25);
//            telemetry.addData("Lift Encoder", liftMotor.getCurrentPosition());
//            telemetry.update();
//        } else if (liftDown == true && liftMotor.getCurrentPosition() >= 29500){
//            liftMotor.setPower(-.25);
//        } else {
//            liftMotor.setPower(0);
//        }
    
}