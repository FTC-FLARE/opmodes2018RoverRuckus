package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "K_TeleOp", group = "Kathy")

public class K_TeleOp extends LinearOpMode {
    DigitalChannel touchSensor;
    DigitalChannel magneticSensor;

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor collectorMotor = null;
    private DcMotor elbowMotor = null;
    private DcMotor sliderMotor = null;
    private DcMotor liftMotor = null;

    static final int SLIDE_INCREMENT = 100;
    static final int ELBOW_INCREMENT = 300;
    static final double COUNTS_PER_MOTOR_REV = (560 * 3);
    static final double ELBOW_GEAR_RATIO = 26/1;
    static final double MAX_ELBOW_FORWARD = (ELBOW_GEAR_RATIO *(COUNTS_PER_MOTOR_REV * .75));
    static final int LIFT_HEIGHT = 1000;

    int lastSlideTarget = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        getDrivetrainHardware();
        getToolHardware();

        waitForStart();

        while (opModeIsActive()) {

            controlDrivetrain();
            collectorControl();
            elbowControl();
            slideControl();
            liftControl();

            telemetry.update();
        }
    }

    private void controlDrivetrain() {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        frontLeftPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        backLeftPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        backRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    private void collectorControl() {
        double collectorPower;

        boolean collectorForward = gamepad2.y;
        boolean collectorBackwards = gamepad2.a;

        if (collectorForward) {
            collectorPower = 1;
        } else if (collectorBackwards) {
            collectorPower = -1;
        } else {
            collectorPower = 0;
        }

        collectorMotor.setPower(collectorPower);
    }

    private void elbowControl() {
        int elbowTarget;
        int elbowCurrent = elbowMotor.getCurrentPosition();
        double elbowPower = -gamepad2.right_stick_y;

        if (elbowCurrent >= -1000 && (elbowPower < 0) || elbowCurrent <= 5500 && (elbowPower > 0)) {
            elbowTarget = elbowMotor.getCurrentPosition() + (int)(elbowPower * ELBOW_INCREMENT);
            telemetry.addData("Elbow - Moving", (int)(elbowPower * ELBOW_INCREMENT));

        }else {
            elbowTarget = elbowMotor.getCurrentPosition();
            telemetry.addLine("Elbow - Stop");
        }

        elbowMotor.setTargetPosition(elbowTarget);
        elbowMotor.setPower(1);

        telemetry.addData("      - Target", elbowTarget);
        telemetry.addData("      - Current", elbowMotor.getCurrentPosition());
        telemetry.addData("      - Power",elbowPower);
    }
    private void slideControl() {
        int slideTarget;
        double slideStickValue = -gamepad2.left_stick_y;

        if (!isTriggered(touchSensor) && (slideStickValue < 0) || !isTriggered(magneticSensor) && (slideStickValue > 0)) {
            slideTarget = sliderMotor.getCurrentPosition() + (int)(slideStickValue * SLIDE_INCREMENT);
            lastSlideTarget = slideTarget;
            telemetry.addData("Slide - Moving", (int) (slideStickValue * SLIDE_INCREMENT));
        } else {
            slideTarget = lastSlideTarget;
            telemetry.addLine("Slide - Stop");
        }

        sliderMotor.setTargetPosition(slideTarget);
        sliderMotor.setPower(1);

        telemetry.addData("      - Target", slideTarget);
        telemetry.addData("      - Current", sliderMotor.getCurrentPosition());
        telemetry.addData("      - Magnet",isTriggered(magneticSensor)?"Yes":"No");
        telemetry.addData("      - Touch", isTriggered(touchSensor)?"Yes":"No");
    }

    private void liftControl() {
        boolean liftUp = gamepad2.right_bumper;
        boolean liftDown = gamepad2.left_bumper;

        if (liftUp && (liftMotor.getCurrentPosition() <= 0)) {
            liftMotor.setPower(.5);
            telemetry.addData("Raise lift", liftMotor.getCurrentPosition());
        } else if (liftDown && liftMotor.getCurrentPosition() >=  -30000) {
            liftMotor.setPower(-.5);
            telemetry.addData("Lower lift", liftMotor.getCurrentPosition());
        } else {
            liftMotor.setPower(0);
            telemetry.addData("Stop lift", liftMotor.getCurrentPosition());
        }
    }

    public boolean isTriggered (DigitalChannel Sensor) {
        return !Sensor.getState();
    }
    private void getDrivetrainHardware() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void getToolHardware() {
        collectorMotor = hardwareMap.get(DcMotor.class, "collector");
        collectorMotor.setDirection(DcMotor.Direction.FORWARD);

        elbowMotor = hardwareMap.get(DcMotor.class, "elbow");
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);
//        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setTargetPosition(0);

        sliderMotor = hardwareMap.get(DcMotor.class, "slide");
        sliderMotor.setDirection(DcMotor.Direction.REVERSE);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setTargetPosition(0);


        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        magneticSensor = hardwareMap.get(DigitalChannel.class, "magneticSensor");
        magneticSensor.setMode(DigitalChannel.Mode.INPUT);


        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(0);

    }

}