package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MM_Tensorflow {

    private LinearOpMode opMode;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    public MM_Tensorflow(LinearOpMode opMode) {
        this.opMode = opMode;
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
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
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX == -1) {
                        opMode.telemetry.addData("Gold Mineral Position", "Left");
                        goldMineralLocation = "Left";
                    }

                    if ((goldMineralX != -1 && silverMineral1X != -1)) {
                        if (goldMineralX > silverMineral1X) {
                            opMode.telemetry.addData("Gold Mineral Position", "Right");
                            goldMineralLocation = "Right";
                            opMode.telemetry.addData("Gold", goldMineralX);
                            opMode.telemetry.addData("Silver", silverMineral1X);
                        } else if (goldMineralX < silverMineral1X) {
                            opMode.telemetry.addData("Gold Mineral Position", "Center");
                            goldMineralLocation = "Center";
                        }
                    }
                }
                opMode.telemetry.update();
            }
        }
        return goldMineralLocation;
    }
}


