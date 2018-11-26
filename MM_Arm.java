package org.firstinspires.ftc.teamcode.opmodes12833;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MM_Arm {
    private DcMotor slide = null;
    private DcMotor elbow = null;
    private LinearOpMode opMode;

    public MM_Arm(LinearOpMode opMode){
        this.opMode = opMode;
        slide = opMode.hardwareMap.get(DcMotor.class, "slide");
        elbow = opMode.hardwareMap.get(DcMotor.class, "elbow");

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
