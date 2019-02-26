package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot Gold Marker", group = "MM")
//@Disabled
public class MMD_Gold_Marker_Crater extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    private ElapsedTime runtime = new ElapsedTime();
    private String goldMineralLocation = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("Waiting for Init");
        telemetry.update();
        robot.init();
        runtime.reset();

        telemetry.addLine("Press Play to start");
        telemetry.update();

        waitForStart();

       goldMineralLocation = robot.deployAndDetect(); // Detect gold location while lowering from lander
        robot.depotLeaveLander();
        robot.findAndMoveToPic();
        robot.driveAndDumpTeamMarker(goldMineralLocation);
        robot.sampleDepotMineral(goldMineralLocation);
        robot.turnOnArmForAutoAndCollect(1);
    }
}