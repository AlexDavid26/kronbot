package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "Odometrie cu reset ADAPTAT")
public class OdometrieCuResetAdaptat extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            //"Marker"
    };


    private static final String VUFORIA_KEY = "ARfXSg3/////AAABmeAedbcQY08Hq6RpQG93FA8zpqyJCYfLEr4REOzdT2VrPv9iQDD2dR7I2Mj+Q0u1V+nd/binuIaCYlNosxD+UW1F4qOKkG8LtlXIvF0pwVHMbg8pm3apX3RyOWdZEk+Jx9Dnsv6cdIehvgdkNZGleEEIcUxBsO0WXS/pUPrdu/xfEmw61qtKcmrnaRQ+uzTCyhcp9G24swdYg9R6k7OAw93N+DCbYqcib+mD3smZcVnZn7nQDYv0MWJCsGYr5bsAvrH/SMPz7BWeErSGZZkCYFIKkZcrCvS2OHOqQ7Fs4k6cg/mgx8kVNS09hFlR7OX8kjclwFodb/j6Az+R0Q7jMaKudDfT3a9UOQMyw2U7oGea";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private int pozitieRatusca = 3;


    RobotTest robot = new RobotTest();
    ElapsedTime timer = new ElapsedTime();
    //Odometry Wheels
    //DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 685; // constanta calculata. daca da pe langa modif aici
    final double kp = 0.01, kd = 0.005;
    final double kpDistance = 0.05, kpRotation = 0.011;
    final double allowableErrorRotation = 5,  allowableErrorForward = 3;

    double error_old;

    OdometryGlobalCoordinatePositionAdaptat globalPositionUpdate;

    List<Double> powerMotors = new ArrayList<Double>(); // puterile pentru fiecare dintre cele 4 motoare

    DcMotor orizontal;

    private static final double[] distancePositions = {0,0,24};
    private static final double[] armPositions = {0.17,0.31,0.47};
    private static final double[] cupaPositions = {0.18,0.33,1};

    @Override
    public void runOpMode() throws InterruptedException {
        initDriveHardwareMap();
        initVuforia();
        initTfod();

        if (tfod != null)
            tfod.activate();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        robot.arm.setPosition(1);
        robot.cupa.setPosition(0);

        detectareRatusca();

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePositionAdaptat(robot.flMotor, robot.frMotor, orizontal, COUNTS_PER_INCH, 0);
        globalPositionUpdate.reverseNormalEncoder();

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //goToPosition(-50,0.5,globalPositionUpdate.returnOrientation(),3);

        goToPosition(-97,0.5,0,allowableErrorForward);
        //goForward(-93, 0.8, 3);
        if(!opModeIsActive())
            return;

        rotateToDegree(90, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        putCubeOnLevel();
        if(!opModeIsActive())
            return;

        rotateToDegree(27, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        goToPosition(103, 0.5, 27, allowableErrorForward);
        if(!opModeIsActive())
            return;

        rotateToDegree(-62, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        robot.drive(1,-1,-1,1,0.2);
        wait_seconds(0.6);
        if(!opModeIsActive())
            return;
        robot.stopMotors();

        robot.squishy.setPower(-0.8);
        wait_seconds(4);
        robot.squishy.setPower(0);
        if(!opModeIsActive())
            return;

        rotateToDegree(0, allowableErrorRotation, 0.5);
        if(!opModeIsActive())
            return;

        goToPosition(-58,0.5,0,allowableErrorForward);


        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.flMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.frMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", orizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap() {

        robot.flMotor = hardwareMap.dcMotor.get("fl");
        robot.frMotor = hardwareMap.dcMotor.get("fr");
        robot.blMotor = hardwareMap.dcMotor.get("bl");
        robot.brMotor = hardwareMap.dcMotor.get("br");
        robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        orizontal = hardwareMap.dcMotor.get("orizontal");
        orizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        orizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.arm = hardwareMap.servo.get("arm");
        robot.cupa = hardwareMap.servo.get("cupa");
        robot.squishy = hardwareMap.dcMotor.get("squishy");

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }


    double GetMinErrorAngle(double currOrientation, double desireOrientation) {

        double beta = (360 - currOrientation + desireOrientation) % 360;
        double gama = beta - 360;

        if (Math.abs(beta) < Math.abs(gama))
            return beta;
        else return gama;

    }

    double PIDAngle(double desire, double act){

        double error_act = GetMinErrorAngle(desire, act);
        double valToReturn = error_act * kp + (error_act - error_old) * kd;
        error_old = error_act;
        return valToReturn;
    }

    double PIDistance(double distance) {
        double valToReturn = distance * kpDistance;
        return valToReturn;
    }

    double PIDRotation(double rotation) {
        double valToReturn = rotation * kpRotation;
        return valToReturn;
    }

    public void goForward(double targetYPosition, double robotPower, double allowableDistanceError) {
        globalPositionUpdate.resetEncoders();

        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());
        error_old=0;

        while (opModeIsActive() && distanceToYTarget > allowableDistanceError) {
            distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

            double pidistance = PIDistance(distanceToYTarget / COUNTS_PER_INCH);

            if (pidistance > 1)
                pidistance = 1;

            pidistance = Map(pidistance, 0, 1, 0, robotPower);

            pidistance = newSpeed(pidistance,robotPower);

            if (targetYPosition < globalPositionUpdate.returnYCoordinate())
                pidistance *= -1;

            if(pidistance==0)
                break;

            robot.DriveWithSpeeds(pidistance, pidistance, pidistance, pidistance);

            telemetry.addData("Distance to Target:", distanceToYTarget / COUNTS_PER_INCH);
            telemetry.addData("allowable distance error: ", allowableDistanceError / COUNTS_PER_INCH);
            telemetry.addData("coord x:", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("coord y:", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.update();
        }

        telemetry.addData("Distance to target: ", "Target Reached");
        telemetry.update();
        robot.stopMotors();
    }

    public void goToPosition(double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError)
    {
        globalPositionUpdate.resetEncoders();

        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

        while (opModeIsActive() && distanceToYTarget > allowableDistanceError) {
            distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

            double pidangle = PIDAngle(desiredRobotOrientation, globalPositionUpdate.returnOrientation());
            double pidistance = PIDistance(distanceToYTarget / COUNTS_PER_INCH);

            telemetry.addData("PidAngle", pidangle);

            if(pidistance > 1)
                pidistance = 1;
            pidistance = Map(pidistance, 0, 1, 0, robotPower);
            //telemetry.addData("Pidistance dupa map", pidistance);

            if (targetYPosition < globalPositionUpdate.returnYCoordinate())
                pidistance *= -1;

            double leftMotorSpeed = pidistance - pidangle;
            double rightMotorSpeed = pidistance + pidangle;

            leftMotorSpeed = newSpeed(leftMotorSpeed, robotPower);
            rightMotorSpeed = newSpeed(rightMotorSpeed, robotPower);

            robot.DriveWithSpeeds(leftMotorSpeed, rightMotorSpeed, leftMotorSpeed, rightMotorSpeed);

            telemetry.addData("Left power:", leftMotorSpeed);
            telemetry.addData("Right power:", rightMotorSpeed);
            telemetry.addData("Distance to Target:", distanceToYTarget/COUNTS_PER_INCH);
            telemetry.addData("allowable distance error: ", allowableDistanceError/COUNTS_PER_INCH);
            telemetry.addData("coord x:", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("coord y:", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.update();
        }

        telemetry.addData("Distance to target: ", "Target Reached");
        telemetry.update();
        robot.stopMotors();
    }

    public void rotateToDegree(double targetDegree, double allowableAngleError, double robotRotationPower) {
        double currentOrientation = globalPositionUpdate.returnOrientation();
        telemetry.addData("valoare", Math.abs(targetDegree - currentOrientation));

        while (opModeIsActive() && Math.abs(targetDegree - currentOrientation) > allowableAngleError) {
            double pidRotation = PIDRotation(GetMinErrorAngle(currentOrientation, targetDegree));
            telemetry.addData("current Orientation:", currentOrientation);
            telemetry.addData("pidRotation", pidRotation);
            double leftMotorSpeed = pidRotation;
            double rightMotorSpeed = -pidRotation;

            leftMotorSpeed = newSpeed(leftMotorSpeed, robotRotationPower);
            rightMotorSpeed = newSpeed(rightMotorSpeed, robotRotationPower);
            if(leftMotorSpeed == 0 || rightMotorSpeed == 0)
                break;
            telemetry.addData("leftSpeed", leftMotorSpeed);
            telemetry.addData("rightSpeed", rightMotorSpeed);
            telemetry.update();
            currentOrientation = globalPositionUpdate.returnOrientation();
            robot.DriveWithSpeeds(leftMotorSpeed, rightMotorSpeed, leftMotorSpeed, rightMotorSpeed);

        }
        robot.drive(1, 1, 1, 1, 0);

    }

    void putCubeOnLevel(){
        goToPosition(-distancePositions[pozitieRatusca-1],0.6,90,allowableErrorForward);
        if(!opModeIsActive())
            return;
        robot.arm.setPosition(armPositions[pozitieRatusca-1]);

        wait_seconds(2.5);
        if(!opModeIsActive())
            return;

        robot.cupa.setPosition(cupaPositions[pozitieRatusca-1]);

        wait_seconds(1.5);
        if(!opModeIsActive())
            return;

        robot.arm.setPosition(1);

        if(!opModeIsActive())
            return;

        goToPosition(distancePositions[pozitieRatusca-1],0.6,90,allowableErrorForward);
    }

    double newSpeed(double vit, double robotSpeed) {
        if (vit > robotSpeed)
            return robotSpeed;
        if (Math.abs(vit) < 0.1)
            return 0;
        if (vit < -robotSpeed)
            return -robotSpeed;
        return vit;
    }

    double Map(double x, double in_min, double in_max, double out_min, double out_max) {

        double rez = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return rez;

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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private int calculatePosition(float right, float left)
    {
        if ((left + right)/2 < 300)
            return 1;
        return 2;
    }

    private void detectareRatusca(){
        while (!opModeIsActive())
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null && !updatedRecognitions.isEmpty()) {
                    Recognition recognition = updatedRecognitions.get(0);
                    pozitieRatusca = calculatePosition(recognition.getRight(), recognition.getLeft());
                }
                else
                    pozitieRatusca = 3;

                telemetry.addData("Pozitie", pozitieRatusca);
                telemetry.update();
            }

        telemetry.addData("Pozitie", pozitieRatusca);
        telemetry.update();
    }

    void wait_seconds(double seconds){
        timer.reset();
        while (timer.seconds() < seconds && opModeIsActive()) ;
    }
}
