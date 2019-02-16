package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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

    DigitalChannel liftMagnetBottom;
    DigitalChannel liftMagnetTop;

    private DcMotor liftMotor = null;

    static final int SLIDE_INCREMENT = 250;
    static final int ELBOW_INCREMENT = 700;
    static final double COUNTS_PER_MOTOR_REV = (560 * 3);
    static final double ELBOW_GEAR_RATIO = 26/1;
    static final double MAX_ELBOW_FORWARD = (ELBOW_GEAR_RATIO *(COUNTS_PER_MOTOR_REV * .75));
    static final int LIFT_HEIGHT = 1000;
    static final double PHONE_UP = .85;

    int lastSlideTarget = 0;
    private boolean driveSlow = true;
    private boolean isButtonAHandled = false;

    private LinearOpMode opMode;
    private Servo phoneTilt = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        getDrivetrainHardware();
        getToolHardware();

        movePhoneUp();

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

        if (gamepad1.a && !isButtonAHandled) {
            driveSlow = !driveSlow;
            isButtonAHandled = true;
        } else if (!gamepad1.a) {
            isButtonAHandled = false;
        }

        if (driveSlow) {
            frontLeftPower /= 2;
            frontRightPower /= 2;
            backLeftPower /= 2;
            backRightPower /= 2;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    private void collectorControl() {
        double collectorPower;

        boolean collectorForward = gamepad2.left_trigger > 0;
        boolean collectorBackwards = gamepad2.right_trigger > 0;

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
        double slideStickValue = (gamepad2.left_stick_y == 0 ? 0 : (gamepad2.left_stick_y < 0 ? 1 : -1)) ;

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

        if (liftUp && !isTriggered(liftMagnetTop)) {
            liftMotor.setPower(1);
            telemetry.addData("Raise lift", liftMotor.getCurrentPosition());
        } else if (liftDown && !isTriggered(liftMagnetBottom)) {
            liftMotor.setPower(-1);
            telemetry.addData("Lower lift", liftMotor.getCurrentPosition());
        } else {
            liftMotor.setPower(0);
            telemetry.addData("Stop lift", liftMotor.getCurrentPosition());
        }
        telemetry.addData("Magnet Bottom",!liftMagnetBottom.getState());
        telemetry.addData("Magnet Top", !liftMagnetTop.getState());
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

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(0);

        liftMagnetBottom = hardwareMap.get(DigitalChannel.class, "liftMagnetBottom");
        liftMagnetBottom.setMode(DigitalChannel.Mode.INPUT);
        liftMagnetTop = hardwareMap.get(DigitalChannel.class, "liftMagnetTop");
        liftMagnetTop.setMode(DigitalChannel.Mode.INPUT);

        phoneTilt = hardwareMap.get(Servo.class, "phoneTilt");
    }
    public void movePhoneUp () {
        phoneTilt.setPosition(PHONE_UP);
    }
}
