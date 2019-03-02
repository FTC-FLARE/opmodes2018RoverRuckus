package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot Gold Marker Alternative", group = "MM")
//@Disabled
public class MMD_Gold_Marker_Crater_Alternative extends LinearOpMode {
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
        robot.depotLeaveLander("alternative");

        if (!goldMineralLocation.equals("Right")) {
            leftAndCenter();
        }
        else{
            right();
        }
    }

    public void leftAndCenter(){
            robot.findAndMoveToAltPic(30); //drives entered amount of inches
            robot.driveAndDumpTeamMarker(goldMineralLocation);
            robot.sampleAltDepotMineral(goldMineralLocation);
            robot.goToOpponentCrater(goldMineralLocation);
    }
    public void right(){
        robot.drivetrain.gyroTurn(.6,98);
        robot.drivetrain.backward(1, 15, 5);
        robot.scoreMineral();
        robot.drivetrain.forward(1, 15, 5);
        robot.drivetrain.gyroTurn(.6,116);
        robot.findAndMoveToAltPic(30); //drives entered amount of inches
        robot.driveAndDumpTeamMarker(goldMineralLocation);
        robot.goToOpponentCrater(goldMineralLocation);

    }
}