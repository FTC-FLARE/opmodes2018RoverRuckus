package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MM_Tensorflow {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    private double goldMineralX = -1;
    private double silverMineralX = -1;

    public MM_Tensorflow(LinearOpMode opMode) {
        this.opMode = opMode;
        initVuforia();
        initTfod();
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public String detectGoldMineral() {
        String goldMineralLocation = "";
        if (tfod != null) {
            tfod.activate();
        }

        runtime.reset();

        if (tfod != null) {
            // keep checking until we determine where gold is || we go past 5 seconds || we hit stop
            while (goldMineralLocation.equals("") && opMode.opModeIsActive() && runtime.seconds() < 5) {
                goldMineralLocation = mineForGold();
                opMode.telemetry.update();
            }
            if (goldMineralLocation.equals("")) {
                return "Left";
            }
            return goldMineralLocation;
        }
        return "Left";
    }

    public String mineForGold() {
        String goldMineralLocation = "";
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 2) { // if we detected 2 and only 2 minerals
                goldMineralLocation = determineGoldPosition(updatedRecognitions);
            }
            opMode.telemetry.addData("Gold Mineral Position",goldMineralLocation);
        }
        return goldMineralLocation;
    }

    private String determineGoldPosition(List<Recognition> updatedRecognitions) {
        String goldMineralLocation;
        for (Recognition recognition : updatedRecognitions) { //runs through logic for each mineral detected
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralX = recognition.getLeft();
            } else {
                silverMineralX = recognition.getLeft();
            }
        }
        if (getGoldMineralX() == -1) { // Only see two silver minerals
            goldMineralLocation = "Left";
        } else { // We see a gold mineral
            if (getGoldMineralX() > getSilverMineralX()) { // Determine gold mineral location
                goldMineralLocation = "Right";
            } else {
                goldMineralLocation = "Center";
            }
        }
        opMode.telemetry.addData("GoldX", getGoldMineralX());
        opMode.telemetry.addData("SilverX", getSilverMineralX());
        return goldMineralLocation;
    }

    public double getGoldMineralX() {
        return goldMineralX;
    }

    public double getSilverMineralX() {
        return silverMineralX;
    }
}