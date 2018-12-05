package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "K_TeleOp_Manual", group = "Kathy")
//@Disabled
public class K_TeleOp_Manual extends LinearOpMode {

    private DcMotor driveFrontLeft = null;
    private DcMotor driveFrontRight = null;
    private DcMotor driveBackLeft = null;
    private DcMotor driveBackRight = null;

    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;

    private DcMotor collector = null;
    private DcMotor armFlipper = null;
    private DcMotor slide = null;
    private DcMotor lift = null;

    @Override
    public void runOpMode() {

        driveFrontLeft = hardwareMap.get(DcMotor.class, "flMotor");
        driveFrontRight = hardwareMap.get(DcMotor.class, "frMotor");
        driveBackLeft = hardwareMap.get(DcMotor.class, "blMotor");
        driveBackRight = hardwareMap.get(DcMotor.class, "brMotor");

        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);

        collector = hardwareMap.get(DcMotor.class, "collector");  // Tetrix
        armFlipper = hardwareMap.get(DcMotor.class, "elbow");
        armFlipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Press Start when ready." );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            // control mineral collector (Tetrix motor)
            if (gamepad2.y) { // forward
                collector.setPower(1.0);
            }
            else if (gamepad2.a){  // reverse
                collector.setPower(-1.0);
            }
            else if (gamepad2.x){ // stop
                collector.setPower(0);
            }

            if (gamepad2.left_bumper)
                lift.setPower(-1);
            else if (gamepad2.right_bumper)
                lift.setPower(1);
            else
                lift.setPower(0);

            // control arm flipper
            armFlipper.setPower(-gamepad2.right_stick_y * .50);

            slide.setPower(-gamepad2.left_stick_y * .50);

        // ***** drivetrain control
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            frontLeftPower = drive + strafe + rotate;
            frontRightPower = drive - strafe - rotate;
            backLeftPower = drive - strafe + rotate;
            backRightPower = drive + strafe - rotate;

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            driveFrontLeft.setPower(frontLeftPower);
            driveFrontRight.setPower(frontRightPower);
            driveBackLeft.setPower(backLeftPower);
            driveBackRight.setPower(backRightPower);
    // ***** end drivetrain control

            telemetry.addData("Arm Flipper Power", armFlipper.getPower());
            telemetry.addData("Slide Power", slide.getPower());
            telemetry.addData("Drivetrain", " %7d :%7d : %7d : %7d", driveFrontLeft.getCurrentPosition(), driveFrontRight.getCurrentPosition(), driveBackLeft.getCurrentPosition(), driveBackRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
