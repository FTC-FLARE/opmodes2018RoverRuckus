package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MM_DriveTrain {

    private BNO055IMU imu;

    private DcMotor flMotor = null;
    private DcMotor frMotor = null;
    private DcMotor blMotor = null;
    private DcMotor brMotor = null;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // 40:1
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode;
    private Orientation angles;

    private double frontLeftPowerForVuforia = 0;
    private double frontRightPowerForVuforia = 0;
    private double backLeftPowerForVuforia = 0;
    private double backRightPowerForVuforia = 0;

    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro

    public MM_DriveTrain(LinearOpMode opMode){
        this.opMode = opMode;
        flMotor = opMode.hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotor.class, "brMotor");

        flMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        blMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        brMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        setMotorPowerSame(0);

        initializeGyro();
        initHardware();
    }
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, double timeoutS) {
        if (opMode.opModeIsActive()) {

            setNewMotorTarget(frontLeftInches, frontRightInches, backLeftInches, backRightInches);
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            setMotorPowerSame(speed);

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (flMotor.isBusy() && frMotor.isBusy() && blMotor.isBusy() && brMotor.isBusy())) {
                encoderTelemetry();
            }
            setMotorPowerSame(0);
            setUsingEncoder();
        }
    }
    public void forward(double speed, double inches, double timeOutS){
        encoderDrive(speed, inches, inches, inches, inches, timeOutS);
    }
    public void backward(double speed, double inches, double timeOutS){
        encoderDrive(speed, -inches, -inches, -inches, -inches, timeOutS);
    }
    public void strafeRight(double speed, double inches, double timeOutS){
        encoderDrive(speed, inches, -inches, -inches, inches, timeOutS);
    }
    public void strafeLeft(double speed, double inches, double timeOutS){
        encoderDrive(speed, -inches, inches, inches, -inches, timeOutS);
    }
    private void encoderTelemetry() {
        opMode.telemetry.addData("fl current", flMotor.getCurrentPosition());
        opMode.telemetry.addData("   target", flMotor.getTargetPosition());

        opMode.telemetry.addData("fr current", frMotor.getCurrentPosition());
        opMode.telemetry.addData("   target", frMotor.getTargetPosition());

        opMode.telemetry.addData("bl current", blMotor.getCurrentPosition());
        opMode.telemetry.addData("   target", blMotor.getTargetPosition());

        opMode.telemetry.addData("br current", brMotor.getCurrentPosition());
        opMode.telemetry.addData("   target", brMotor.getTargetPosition());

        opMode.telemetry.update();
    }

    private void setNewMotorTarget(double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches) {
        int newFrontLeftTarget = flMotor.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
        int newFrontRightTarget = frMotor.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
        int newBackLeftTarget = blMotor.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
        int newBackRightTarget = brMotor.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);

        setEncoderTargets(newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
    }

    private void setMotorPowerSame(double speed) {
        setMotorPowers(speed, speed, speed, speed);
    }

    private void setEncoderTargets(int newFrontLeftTarget, int newFrontRightTarget, int newBackLeftTarget, int newBackRightTarget) {
        flMotor.setTargetPosition(newFrontLeftTarget);
        frMotor.setTargetPosition(newFrontRightTarget);
        blMotor.setTargetPosition(newBackLeftTarget);
        brMotor.setTargetPosition(newBackRightTarget);
    }
    public void setMotorPowers(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        flMotor.setPower(flSpeed);
        frMotor.setPower(frSpeed);
        blMotor.setPower(blSpeed);
        brMotor.setPower(brSpeed);
    }

    public void stopMotors(){
        setMotorPowerSame(0);
    }

    public void initHardware() {
        stopAndResetEncoders();
        setUsingEncoder();
    }

    private void setUsingEncoder() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAndResetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void brakesOn() {
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorMode(DcMotor.RunMode stopAndResetEncoder) {
        flMotor.setMode(stopAndResetEncoder);
        frMotor.setMode(stopAndResetEncoder);
        blMotor.setMode(stopAndResetEncoder);
        brMotor.setMode(stopAndResetEncoder);
    }

    public void initializeGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
    public void gyroTurn (double speed, double angle) {
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            opMode.telemetry.addData("current angle", angles.firstAngle);
            opMode.telemetry.update();
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
        flMotor.setPower(leftSpeed);
        frMotor.setPower(rightSpeed);
        blMotor.setPower(leftSpeed);
        brMotor.setPower(rightSpeed);

        // Display it for the driver.
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void moveRobot() {
        flMotor.setPower(setMinPower(frontLeftPowerForVuforia * 0.6));
        frMotor.setPower(setMinPower(frontRightPowerForVuforia * 0.6));
        blMotor.setPower(setMinPower(backLeftPowerForVuforia * 0.6));
        brMotor.setPower(setMinPower(backRightPowerForVuforia * 0.6));

        opMode.telemetry.addData("Wheels", "FL[%+5.2f], FR[%+5.2f], BL[%+5.2f], BR[%+5.2f]", frontLeftPowerForVuforia, frontRightPowerForVuforia, backLeftPowerForVuforia, backRightPowerForVuforia);
    }
    public double setMinPower(double inPower) {
        if (inPower < .15 && inPower > 0) {
            return inPower + .05;
        }
        if (inPower > -.15 && inPower < 0) {
            return inPower - .05;
        }
        return inPower;
    }

    public double getCurrentHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }
    public void setFrontLeftPowerForVuforia(double frontLeftPowerForVuforia) {
        this.frontLeftPowerForVuforia = frontLeftPowerForVuforia;
    }

    public void setFrontRightPowerForVuforia(double frontRightPowerForVuforia) {
        this.frontRightPowerForVuforia = frontRightPowerForVuforia;
    }

    public void setBackLeftPowerForVuforia(double backLeftPowerForVuforia) {
        this.backLeftPowerForVuforia = backLeftPowerForVuforia;
    }

    public void setBackRightPowerForVuforia(double backRightPowerForVuforia) {
        this.backRightPowerForVuforia = backRightPowerForVuforia;
    }
}
