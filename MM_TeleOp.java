package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MMTeleOp", group = "Ian Hunter")

public class MM_TeleOp extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    DigitalChannel touchSensor;
    DigitalChannel magneticSensor;

//    private DcMotor flMotor = null;
//    private DcMotor frMotor = null;
//    private DcMotor blMotor = null;
//    private DcMotor brMotor = null;
    private DcMotor collectorMotor = null;
    private DcMotor elbowMotor = null;
    private DcMotor sliderMotor = null;
    private DcMotor liftMotor = null;

    static final double COUNTS_PER_MOTOR_REV = (560 * 3);
    static final double ELBOW_GEAR_RATIO = 26/1;
    static final double MAX_ELBOW_FORWARD = (ELBOW_GEAR_RATIO *(COUNTS_PER_MOTOR_REV * .75));
    static final double POWER_INCREMENT = 0.5;
    static final int LIFT_HEIGHT = 1000;


    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        magneticSensor = hardwareMap.get(DigitalChannel.class, "magneticSensor");
        magneticSensor.setMode(DigitalChannel.Mode.INPUT);

//        digitalTouchDown = hardwareMap.get(DigitalChannel.class, "sensor_digital");
//        digitalTouchDown.setMode(DigitalChannel.Mode.INPUT);
//
//        digitalTouchUp = hardwareMap.get(DigitalChannel.class, "sensor_digital");
//        digitalTouchUp.setMode(DigitalChannel.Mode.INPUT);

        collectorMotor = hardwareMap.get(DcMotor.class, "collector");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbow");
        sliderMotor = hardwareMap.get(DcMotor.class, "slide");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");

       sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectorMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);
        sliderMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double slideTarget;
        double elbowTarget;

        while (opModeIsActive()) {

            robot.drivetrain.driveWithSticks();

/*
            slideControl();
            elbowControl();
            liftMech();
*/

/*
            boolean collectorForward = gamepad2.y;
            boolean collectorBackwards = gamepad2.a;

            double collectorPower;
            if (collectorForward) {
                collectorPower = 1;
            } else if (collectorBackwards) {
                collectorPower = -1;
            } else {
                collectorPower = 0;
            }
            collectorMotor.setPower(collectorPower);
*/

            telemetry.addData("SliderEncoder","%5d",sliderMotor.getCurrentPosition());
            telemetry.addData("ElbowEncoder","%5d",elbowMotor.getCurrentPosition());
            telemetry.addData("LiftEncoder","%5d",liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    private void slideControl() {
        int slideTarget;
        double slide = -gamepad2.left_stick_y;

        if (!isTriggered(touchSensor) && (slide < 0) || !isTriggered(magneticSensor) && (slide > 0)) {
            slideTarget = sliderMotor.getCurrentPosition() + (int)(slide * POWER_INCREMENT);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setTargetPosition(slideTarget);
            sliderMotor.setPower(1);
        } else {
            slideTarget = sliderMotor.getCurrentPosition();
            sliderMotor.setPower(.25);
            telemetry.addData("Stop slide",slide);
        }
        telemetry.addData("Magnetic Sensor",magneticSensor.getState());
        telemetry.addData("Current Sensor", isTriggered(touchSensor));
        telemetry.addData("Current Slide Position", sliderMotor.getCurrentPosition());
        telemetry.addData("Slide Target Position", slideTarget);
        telemetry.addData("Joystick Position",slide);

    }

    private void elbowControl() {
        int elbowTarget;
        double elbowPower = -gamepad2.right_stick_y;
        if (elbowMotor.getCurrentPosition() >= 0 && elbowMotor.getCurrentPosition() <= 5500) {
//            elbowMotor.setPower(elbowPower);
            elbowTarget = elbowMotor.getCurrentPosition() + (int)(elbowPower * POWER_INCREMENT);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(elbowTarget);
            elbowMotor.setPower(1);
        } else if(elbowMotor.getCurrentPosition() > 5500 && elbowPower <0) {
//            elbowMotor.setPower(elbowPower);
            elbowTarget = elbowMotor.getCurrentPosition() + (int)(elbowPower * POWER_INCREMENT);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(elbowTarget);
            elbowMotor.setPower(1);
        }else if(elbowMotor.getCurrentPosition() < 0 && elbowPower > 0) {
//            elbowMotor.setPower(elbowPower);
            elbowTarget = elbowMotor.getCurrentPosition() + (int)(elbowPower * POWER_INCREMENT);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(elbowTarget);
            elbowMotor.setPower(1);
        }else {
            elbowTarget = elbowMotor.getCurrentPosition();
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(elbowTarget);
            elbowMotor.setPower(.25);
        }
        telemetry.addData("Elbow Encoder", elbowMotor.getCurrentPosition());
        telemetry.addData("Elbow Power",elbowPower);
    }
    private void liftMech() {
        boolean liftUp = gamepad2.x;
        boolean liftDown = gamepad2.b;
        double stick = -gamepad2.left_stick_y;

//            liftMotor.setPower(stick);
//            telemetry.addData("Lift Encoder", liftMotor.getCurrentPosition());
//            telemetry.update();

        if (liftUp == true && (liftMotor.getCurrentPosition() <= 29500 && liftMotor.getCurrentPosition() >= 0)) {
            liftMotor.setPower(.25);
            telemetry.addData("Lift Encoder", liftMotor.getCurrentPosition());
            telemetry.update();
        } else if (liftDown == true && liftMotor.getCurrentPosition() >= 29500) {
            liftMotor.setPower(-.25);
        } else {
            liftMotor.setPower(0);
        }
    }

    public boolean isTriggered (DigitalChannel Sensor) {
        return !Sensor.getState();
    }
}