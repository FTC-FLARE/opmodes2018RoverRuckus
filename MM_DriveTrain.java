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

    static final double COUNTS_PER_MOTOR_REV = 1680;    // 60:1
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_POWER = 0.6;
    static final double STRAFE_POWER = 1;

    private ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode;
    private Orientation angles;

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
//        composeTelemetry();
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
                    (flMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy())) {
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
    private void setMotorPowers(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        flMotor.setPower(flSpeed);
        frMotor.setPower(frSpeed);
        blMotor.setPower(blSpeed);
        brMotor.setPower(brSpeed);
    }

    public void strafeRightPower(double power) {
        setMotorPowers(power, -power, -power, power);
    }
    public void strafeLeftPower(double power) {
        setMotorPowers(-power, power, power, -power);
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
    public void driveUntilCrater(){
        runtime.reset();
        setMotorPowerSame(DRIVE_POWER);
        while ((angles.secondAngle) > -3.0 && opMode.opModeIsActive()){
            opMode.telemetry.addData("Angle", angles.secondAngle);
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }
    public void strafeRightUntilCrater(){
        runtime.reset();
        strafeRightPower(STRAFE_POWER);
        while ((angles.thirdAngle) > -3.5 && opMode.opModeIsActive() && runtime.seconds() < 2){
            opMode.telemetry.addData("Angle", angles.thirdAngle);
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }
    public void strafeLeftUntilCrater(){
        runtime.reset();
        strafeLeftPower(STRAFE_POWER);
        while ((angles.secondAngle) > -3.0 && opMode.opModeIsActive()){
            opMode.telemetry.addData("Angle", angles.secondAngle);
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }
    public void driveUntilDepot() {
        runtime.reset();
        setMotorPowerSame(DRIVE_POWER);
        while (runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Time", runtime.seconds());
            opMode.telemetry.update();
        }
        stopMotors();
    }

    void composeTelemetry() {

        opMode.telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
        opMode.telemetry.addData("heading", angles.firstAngle);
        opMode.telemetry.addData("roll", angles.secondAngle);
        opMode.telemetry.addData("pitch", angles.thirdAngle);
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
    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;

        double frontLeftPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        double frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        double backLeftPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        double backRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

        flMotor.setPower(frontLeftPower);
        frMotor.setPower(frontRightPower);
        blMotor.setPower(backLeftPower);
        brMotor.setPower(backRightPower);

        opMode.telemetry.addData("","flPower %.2f - frPower %.2f - blPower %.2f - brPower %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}
