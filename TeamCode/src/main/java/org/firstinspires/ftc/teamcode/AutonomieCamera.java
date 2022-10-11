package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Autonomie Camera")
public class AutonomieCamera extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {
            "TeamElement"
    };

    private static final String VUFORIA_KEY = "ARfXSg3/////AAABmeAedbcQY08Hq6RpQG93FA8zpqyJCYfLEr4REOzdT2VrPv9iQDD2dR7I2Mj+Q0u1V+nd/binuIaCYlNosxD+UW1F4qOKkG8LtlXIvF0pwVHMbg8pm3apX3RyOWdZEk+Jx9Dnsv6cdIehvgdkNZGleEEIcUxBsO0WXS/pUPrdu/xfEmw61qtKcmrnaRQ+uzTCyhcp9G24swdYg9R6k7OAw93N+DCbYqcib+mD3smZcVnZn7nQDYv0MWJCsGYr5bsAvrH/SMPz7BWeErSGZZkCYFIKkZcrCvS2OHOqQ7Fs4k6cg/mgx8kVNS09hFlR7OX8kjclwFodb/j6Az+R0Q7jMaKudDfT3a9UOQMyw2U7oGea";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private int pozitie = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();

        if (tfod != null)
            tfod.activate();

        detectare();

        waitForStart();
    }

    private int calculatePosition(float right, float left) {
        if (left + right < 500)
            return 1;
        return 2;
    }

    private void detectare() {
        while (!opModeIsActive())
            if (tfod != null) {
                telemetry.addData("Pozitie", pozitie);
                telemetry.update();
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && !updatedRecognitions.isEmpty()) {
                    Recognition recognition = updatedRecognitions.get(0);
                    pozitie = calculatePosition(recognition.getRight(), recognition.getLeft());
                } else
                    pozitie = 3;
            }

        telemetry.addData("Pozitie", pozitie);
        telemetry.update();
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
