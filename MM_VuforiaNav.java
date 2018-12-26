package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class MM_VuforiaNav {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    private VuforiaLocalizer vuforia;

    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    public MM_VuforiaNav(LinearOpMode opMode) {
        this.opMode = opMode;
        initVuforia();
    }

    public void initVuforia() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }
}