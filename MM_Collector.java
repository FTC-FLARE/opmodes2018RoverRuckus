package org.firstinspires.ftc.teamcode.opmodes12833;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MM_Collector {
    private DcMotor collector = null;
    private LinearOpMode opMode;

    public MM_Collector(LinearOpMode opMode){
        this.opMode = opMode;
        collector = opMode.hardwareMap.get(DcMotor.class, "collector");
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setCollector(double speed) {
        collector.setPower(speed);
        while(opMode.opModeIsActive() && collector.isBusy());
    }
}
