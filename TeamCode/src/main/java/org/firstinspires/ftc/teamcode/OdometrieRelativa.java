package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Map;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "Odometrie Relativa")
public class OdometrieRelativa extends LinearOpMode {


    RobotTest robot = new RobotTest();
    //Odometry Wheels
    //DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 685; // constanta calculata. daca da pe langa modif aici
    final double kp = 0.05, kd = 0.01;
    final double kpDistance = 0.05, kpRotation = 0.016;

    double error_old;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    List<Double> powerMotors = new ArrayList<Double>(); // puterile pentru fiecare dintre cele 4 motoare

    DcMotor orizontal;

    @Override
    public void runOpMode() throws InterruptedException {

        initDriveHardwareMap();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.flMotor, robot.frMotor, orizontal, COUNTS_PER_INCH, 0);
        globalPositionUpdate.reverseNormalEncoder();

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // globalPositionUpdate.reverseRightEncoder();  // decomentati daca e nevoie ( dupa ce rulati GLobalPositionUpdateSample dati reverse la encoderele care dau cu minus
        // globalPositionUpdate.reverseNormalEncoder();

        /* functia primeste parametrii
         * targetXposition, targetYPosition -> nr_inch*counts_per_inch
         * robotPower -> puterea cu care merge robotul
         * desiredRobotOrientation -> la ce unghi (FATA DE VERTICALA) trebuie sa ajunga - propun ca la testing sa nu ii dati sa se si roteasca pe drum
         * allowableDistanceError -> la cat de punctul final ne putem opri (eroare admisibila)
         */

        goToPosition(20, 20, 0.5, 3, false);
        rotateToDegree(90, 5, 0.5);

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

    double PIDistance(double distance) {
        double valToReturn = distance * kpDistance;
        return valToReturn;
    }

    double PIDRotation(double rotation) {
        double valToReturn = rotation * kpRotation;
        return valToReturn;
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double allowableDistanceError, boolean rotate180) {
        targetXPosition *= COUNTS_PER_INCH;
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToXTarget = Math.abs(targetXPosition - globalPositionUpdate.returnXCoordinate());
        double distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

        double distanceToTarget = Math.hypot(distanceToXTarget, distanceToYTarget); // hypot = radicalul sumei patratelor = ipotenuza

        while (opModeIsActive() && distanceToTarget > allowableDistanceError) {
            distanceToXTarget = Math.abs(targetXPosition - globalPositionUpdate.returnXCoordinate());
            distanceToYTarget = Math.abs(targetYPosition - globalPositionUpdate.returnYCoordinate());

            distanceToTarget = Math.hypot(distanceToXTarget, distanceToYTarget); // hypot = radicalul sumei patratelor = ipotenuza

            rotateToDegree(giveDegree(globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate(), targetXPosition, targetYPosition, distanceToXTarget, distanceToYTarget, rotate180), 5, 0.5);

            double pidistance = PIDistance(distanceToTarget / COUNTS_PER_INCH);

            if (pidistance > 1)
                pidistance = 1;

            pidistance = Map(pidistance, 0, 1, 0, robotPower);
            //telemetry.addData("Pidistance dupa map", pidistance);

            if (globalPositionUpdate.returnXCoordinate() > targetXPosition)
                pidistance *= -1;

            robot.DriveWithSpeeds(pidistance, pidistance, pidistance, pidistance);

            telemetry.addData("Power:", pidistance);
            telemetry.addData("Distance to Target:", distanceToTarget / COUNTS_PER_INCH);
            telemetry.addData("allowable distance error: ", allowableDistanceError / COUNTS_PER_INCH);
            telemetry.addData("coord x:", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("coord y:", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
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
            telemetry.addData("leftSpeed", leftMotorSpeed);
            telemetry.addData("rightSpeed", rightMotorSpeed);
            telemetry.update();
            currentOrientation = globalPositionUpdate.returnOrientation();
            robot.DriveWithSpeeds(leftMotorSpeed, rightMotorSpeed, leftMotorSpeed, rightMotorSpeed);

        }
        robot.drive(1, 1, 1, 1, 0);

    }

    private double giveDegree(double X, double Y, double Xdest, double Ydest, double Xdist, double Ydist, boolean rotate180) {
        double angleToTarget = Math.atan2(Ydist, Xdist);

        if((X > Xdest && Y < Ydest) || (X < Xdest && Y > Ydest))
            angleToTarget = 180 - angleToTarget;

        if (angleToTarget <= 90)
            angleToTarget = Map(angleToTarget, 0, 90, 90, 0);
        else
            angleToTarget = Map(angleToTarget, 90, 180, 0, -90);

        if (rotate180)
            return 180 + angleToTarget;
        return angleToTarget;
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
}
