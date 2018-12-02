package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@Autonomous(name="Autonomous", group="Ian Hunter")

public class MechanumDrivetrainAuto extends LinearOpMode {

    BNO055IMU imu;

    Orientation angles;

    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor liftMotor = null;

    static final double COUNTS_PER_MOTOR_REV = 1680;
    static final double WHEEL_DIAMETER = 4;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER * 3.1415);

    @Override
    public void runOpMode() {

        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Calibration status",imu.isGyroCalibrated());
        telemetry.addData("Status","Initialized");
        telemetry.update();

        frontLeftMotor = hardwareMap.get(DcMotor.class, "flMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "brMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // pauses until the driver hits the play button
        waitForStart();
        //runs after waitForStart
        //Speed always needs to be positive because I take the absolute value of speed later in the program.

//        LiftMech();
//        encoderDrive(1,-5);
//        encoderStrafe(15,1);
//        encoderTurn(5, 1);
        gyroTurn(1, 90);
    }
    public double getError(double targetAngle) {

        double robotError;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    public void encoderDrive(double speed, int inches) {

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        newFrontLeftTarget = (int)(inches * COUNTS_PER_INCH);
        newBackLeftTarget = (int)(inches * COUNTS_PER_INCH);
        newFrontRightTarget = (int)(inches * COUNTS_PER_INCH);
        newBackRightTarget = (int)(inches * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));
        frontRightMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));

        while (frontRightMotor.isBusy() || frontLeftMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy() && opModeIsActive()) {

        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void gyroTurn (double speed, double angle) {
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            telemetry.addData("current angle", angles.firstAngle);
            telemetry.update();
        }
    }    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backLeftMotor.setPower(leftSpeed);
        backRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    // x and y are speeds not distances so they can only be values from -1 to 1
    public void encoderStrafe (double xInches, double speed) {

        frontLeftMotor.setTargetPosition((int) xInches * (int) COUNTS_PER_INCH);
        frontRightMotor.setTargetPosition((int) -xInches * (int) COUNTS_PER_INCH);
        backLeftMotor.setTargetPosition((int) -xInches * (int) COUNTS_PER_INCH);
        backRightMotor.setTargetPosition((int) xInches * (int) COUNTS_PER_INCH);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy() || backLeftMotor.isBusy() && opModeIsActive()){

        }
    }
    private void LiftMech() {

        liftMotor.setPower(1);
        while (liftMotor.getCurrentPosition() <= 30000 && opModeIsActive()) {
            telemetry.addData("Lift Encoder", liftMotor.getCurrentPosition());
            telemetry.update();
        }
        liftMotor.setPower(0);
    }
    private void encoderTurn(int inches, double speed) {

        frontLeftMotor.setTargetPosition(-inches * (int) COUNTS_PER_INCH);
        frontRightMotor.setTargetPosition(inches * (int) COUNTS_PER_INCH);
        backLeftMotor.setTargetPosition(-inches * (int) COUNTS_PER_INCH);
        backRightMotor.setTargetPosition(inches * (int) COUNTS_PER_INCH);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);

        while (opModeIsActive() && frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {

        }
    }

//    public void spline(int x1,int y1,int x2,int y2,int x3,int y3) {
//        int x = (1-t)^2*x1 + 3*(1-t)^2+x2 +t^2*x3;
//        int y = (1-t)^2*y1 + 3*(1-t)^2+y2+t^2*y3;

//    public void spline(double inches, int turn) {
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        frontLeftPower = Range.clip(inches + (turn / 360), -1,1);
//        frontRightPower = Range.clip(inches - (turn / 360),-1,1);
//        backLeftPower = Range.clip(inches + (turn / 360),-1,1);
//        backRightPower = Range.clip(inches - (turn / 360),-1,1);
//
//        frontLeftMotor.setPower(frontLeftPower);
//        frontRightMotor.setPower(frontRightPower);
//        backLeftMotor.setPower(backLeftPower);
//        backRightMotor.setPower(backRightPower);
//
//        while (angles.firstAngle <= turn) {
//            frontLeftMotor.setTargetPosition(turn);
//            frontRightMotor.setTargetPosition(turn);
//            backLeftMotor.setTargetPosition(turn);
//            backRightMotor.setTargetPosition(turn);
//        }

}