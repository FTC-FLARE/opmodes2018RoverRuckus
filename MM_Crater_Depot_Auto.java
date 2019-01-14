package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater and Depot Auto", group = "MM")
@Disabled
public class MM_Crater_Depot_Auto extends LinearOpMode {
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
        robot.driveAndStrafeMineralLocationForCrater(goldMineralLocation);
        robot.driveFromCraterMineralAndDumpTeamMarkerVuforia(goldMineralLocation);
        robot.drivetrain.backward(1, 65, 20);
    }

}