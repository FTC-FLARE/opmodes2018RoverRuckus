package org.firstinspires.ftc.teamcode.opmodes12833;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Arm {
    private DcMotor slide = null;
    private DcMotor elbow = null;
    private LinearOpMode opMode;
    static final int ARM_POSITION = 2000;

    public MM_Arm(LinearOpMode opMode){
        this.opMode = opMode;
        slide = opMode.hardwareMap.get(DcMotor.class, "slide");
        elbow = opMode.hardwareMap.get(DcMotor.class, "elbow");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void slideToPosition(double speed, int position ) {
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setTargetPosition(ARM_POSITION);
        slide.setPower(speed);
        while (opMode.opModeIsActive() && slide.isBusy());
    }
    public void turnOnArm(double power){
        slide.setPower(power);
    }
}
