package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class MM_DriveTrain{
    BNO055IMU imu;
    Orientation angles;

    public DcMotor frontleft = null;
    public DcMotor frontright = null;
    public DcMotor backleft = null;
    public DcMotor backright = null;

    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double TICKS_PER_INCH = COUNTS_PER_INCH * DRIVE_GEAR_REDUCTION / WHEEL_DIAMETER_INCHES * 3.14159;
    static final double MOTOR_RPM = 160;
    static final double DRIVE_RPS = MOTOR_RPM / 60;
    static final double DRIVE_INCHES_PER_SEC = DRIVE_RPS * WHEEL_DIAMETER_INCHES * Math.PI;

    Acceleration gravity;
    private ElapsedTime runtime = new ElapsedTime();
    String goldMineralLocation = "";
    private LinearOpMode opMode;
    public MM_DriveTrain(LinearOpMode opMode){
        // Define and Initialize Motors
        this.opMode = opMode;
        frontleft = opMode.hardwareMap.get(DcMotor.class, "flMotor");
        frontright = opMode.hardwareMap.get(DcMotor.class, "frMotor");
        backleft = opMode.hardwareMap.get(DcMotor.class, "blMotor");
        backright = opMode.hardwareMap.get(DcMotor.class, "brMotor");

        frontleft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontright.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backright.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

        initializeGyro();
        initHardware();
    }
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontleft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontright.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backleft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backright.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);


            frontleft.setTargetPosition(newFrontLeftTarget);
            frontright.setTargetPosition(newFrontRightTarget);
            backleft.setTargetPosition(newBackLeftTarget);
            backright.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())) {
                opMode.telemetry.addData("current", backleft.getCurrentPosition());
                opMode.telemetry.addData("target", backleft.getTargetPosition());

                opMode.telemetry.update();
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

            // Turn off RUN_TO_POSITION
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void constantPower() {
        frontleft.setPower(.6);
        frontright.setPower(.6);
        backleft.setPower(.6);
        backright.setPower(.6);
    }
    public void strafeRightPower() {
        frontleft.setPower(1);
        frontright.setPower(-1);
        backleft.setPower(-1);
        backright.setPower(1);
    }
    public void strafeLeftPower() {
        frontleft.setPower(.9);
        frontright.setPower(-.9);
        backleft.setPower(-.9);
        backright.setPower(.9);
    }


    public void stopMotors(){
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    public void initHardware() {

        // Send telemetry message to signify robot waiting;
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initializeGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }
    public void driveUntilCrater(){
        runtime.reset();
        constantPower();
        while ((angles.secondAngle) > -3.0 && opMode.opModeIsActive()){
            opMode.telemetry.addData("Angle", angles.secondAngle);
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }
    public void strafeRightUntilCrater(){
        runtime.reset();
        strafeRightPower();
        while ((angles.thirdAngle) > -3.5 && opMode.opModeIsActive() && runtime.seconds() < 2){
            opMode.telemetry.addData("Angle", angles.thirdAngle);
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }
    public void strafeLeftUntilCrater(){
        runtime.reset();
        strafeLeftPower();
        while ((angles.secondAngle) > -3.0 && opMode.opModeIsActive()){
            opMode.telemetry.addData("Angle", angles.secondAngle);
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }
    public void driveUntilDepot(){
        runtime.reset();
        constantPower();
        while (runtime.seconds() < 1.5 && opMode.opModeIsActive()){
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }

    void composeTelemetry() {

        opMode.telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        opMode.telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        opMode.telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        opMode.telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void pushMineralCrater(MM_Test_Bot robot){
        while (goldMineralLocation.equals("")) {
            goldMineralLocation = robot.tensorflow.detectGoldMineral();
        }

        if (goldMineralLocation.equals("Left")) {
            encoderDrive(.7, 18, 18, 18, 18, 20); // drive forward
            strafeRightUntilCrater();
        } else if (goldMineralLocation.equals("Right")) {
            encoderDrive(.7, -18, -18, -18, -18, 20); // drive backwards
            strafeRightUntilCrater();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            strafeRightUntilCrater();
        }

    }
    public void strafeMineralCrater(MM_Test_Bot robot){
        while (goldMineralLocation.equals("")) {
            goldMineralLocation = robot.tensorflow.detectGoldMineral();
        }

        encoderDrive(.9, 13, -13, -13, 13, 20);
        encoderDrive(.9, 6, 6, 6, 6, 20);

        if (goldMineralLocation.equals("Left")) {
            encoderDrive(.9, 18, 18, 18, 18, 20); // drive forward
            strafeRightUntilCrater();
        } else if (goldMineralLocation.equals("Right")) {
            encoderDrive(.9, -18, -18, -18, -18, 20); //
            strafeRightUntilCrater();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            strafeRightUntilCrater();
        }

    }

    public void pushMineralDepot(MM_Test_Bot robot){
        while (goldMineralLocation.equals("")) {
            goldMineralLocation = robot.tensorflow.detectGoldMineral();
        }

        encoderDrive(.7, 16, 16, 16, 16, 20);

        if (goldMineralLocation.equals("Left")) {
            encoderDrive(.7, -18, 18, 18, -18, 20); // strafe
            driveUntilDepot();
        } else if (goldMineralLocation.equals("Right")) {
            encoderDrive(.7, 18, -18, -18, 18, 20); // strafe
            driveUntilDepot();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            driveUntilDepot();
        }

    }
}
