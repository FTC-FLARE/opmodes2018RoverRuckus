package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mineral All Four", group = "MM")
//@Disabled
public class MM_Mineral_All_Four extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    private ElapsedTime runtime = new ElapsedTime();
    private String goldMineralLocation = "";
    @Override
    public void runOpMode() {
        robot.init();
        telemetry.addLine("Done with Init");
        telemetry.update();
        runtime.reset();

        telemetry.addData(">", "Press Play");
        telemetry.update();

        waitForStart();

        goldMineralLocation = robot.lift.deployAndDetect(robot);
        robot.moveAwayFromLander();
        robot.onlyMineralAllFour(goldMineralLocation);
    }

}