package org.firstinspires.ftc.teamcode.opmodes12833;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MM_Latch {
    private DcMotor lift = null;
    private LinearOpMode opMode;

    public MM_Latch(LinearOpMode opMode){
        this.opMode = opMode;
        lift = opMode.hardwareMap.get(DcMotor.class, "lift");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
