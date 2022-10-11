package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {


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
        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseLeftEncoder();
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
        //goToPosition(0*COUNTS_PER_INCH, -20*COUNTS_PER_INCH, 0.5, globalPositionUpdate.returnOrientation(), 3*COUNTS_PER_INCH);
        //goForward(50, 0.5, 3);
        //rotateToDegree(90, 5, 0.5);
        //rotateToDegree(0, 5, 0.5);

        while(opModeIsActive()){
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

    private void initDriveHardwareMap(){

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

        if(Math.abs(beta) < Math.abs(gama))
            return beta;
        else return gama;

    }
    ///TEST ATAN BEFORE UPLOADING
    double PIDAngle(double desire, double act){

        double error_act = GetMinErrorAngle(desire, act);
        double valToReturn = error_act * kp + (error_act - error_old) * kd;
        error_old = error_act;
        return valToReturn;


    }

    double PIDistance(double distance){
        double valToReturn = distance*kpDistance;
        return valToReturn;
    }

    double PIDRotation(double rotation){
        double valToReturn = rotation*kpRotation;
        return valToReturn;
    }

    public void goForward(double targetYPosition, double robotPower, double allowableDistanceError){
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        while(opModeIsActive() && distanceToYTarget > allowableDistanceError) {
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double pidistance = PIDistance(distanceToYTarget / COUNTS_PER_INCH);

            if(pidistance > 1)
                pidistance = 1;
            pidistance = Map(pidistance, 0, 1, 0, robotPower);
            //telemetry.addData("Pidistance dupa map", pidistance);

            pidistance = newSpeed(pidistance, robotPower);

            robot.DriveWithSpeeds(pidistance, pidistance, pidistance, pidistance);


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

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError)
    {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distanceToTarget = Math.hypot(distanceToXTarget, distanceToYTarget); // hypot = radicalul sumei patratelor = ipotenuza
        //old_distance = distanceToTarget;
        while(opModeIsActive() && distanceToTarget>allowableDistanceError) {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            distanceToTarget = Math.hypot(distanceToXTarget, distanceToYTarget); // hypot = radicalul sumei patratelor = ipotenuza

            double angleToTarget = Math.atan2(distanceToYTarget, distanceToXTarget);


            double pidangle = PIDAngle(angleToTarget, globalPositionUpdate.returnOrientation());
            double pidistance = PIDistance(distanceToTarget/COUNTS_PER_INCH);

            telemetry.addData("PidAngle", pidangle);
            if(pidistance > 1)
                pidistance = 1;
            pidistance = Map(pidistance, 0, 1, 0, robotPower);
            //telemetry.addData("Pidistance dupa map", pidistance);
            double leftMotorSpeed = pidistance - pidangle;
            double rightMotorSpeed = pidistance + pidangle;

            leftMotorSpeed = newSpeed(leftMotorSpeed, robotPower);
            rightMotorSpeed = newSpeed(rightMotorSpeed, robotPower);

            robot.DriveWithSpeeds(leftMotorSpeed, rightMotorSpeed, rightMotorSpeed, leftMotorSpeed);

            telemetry.addData("Left power:", leftMotorSpeed);
            telemetry.addData("Right power:", rightMotorSpeed);
            telemetry.addData("Distance to Target:", distanceToTarget/COUNTS_PER_INCH);
            telemetry.addData("allowable distance error: ", allowableDistanceError/COUNTS_PER_INCH);
            telemetry.addData("coord x:", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("coord y:", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.update();
        }

        telemetry.addData("Distance to target: ", "Target Reached");
        telemetry.update();
        robot.drive(1, 1, 1, 1, 0);

        rotateToDegree(desiredRobotOrientation, 5, 0.3);
    }

    public void rotateToDegree(double targetDegree, double allowableAngleError, double robotRotationPower)
    {
        double currentOrientation = globalPositionUpdate.returnOrientation();
        telemetry.addData("valoare", Math.abs(targetDegree - currentOrientation));

        while(opModeIsActive() && Math.abs(targetDegree - currentOrientation) > allowableAngleError)
        {
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

    double newSpeed(double vit, double robotSpeed){
        if(vit>robotSpeed)
            return robotSpeed;
        if(Math.abs(vit)<0.1)
            return 0;
        if(vit<-robotSpeed)
            return -robotSpeed;
        return vit;
    }

    double Map(double x, double in_min, double in_max,double out_min, double out_max){

        double rez=(x - in_min)*(out_max - out_min)/(in_max - in_min)+ out_min;
        return rez;

    }

    private void powerForMotors(double velocityX, double velocityY, double velocityR){//primeste ca param componentele pt trasnl, mers in fata, si rotatie
        List<Double> translateValues = Arrays.asList(  // uitati-va pe git daca astea nu merg
                velocityY + velocityX,   // ca se mearga in fata, toate mot cu plus(de-aia sunt velocityY cu plus
                velocityY - velocityX,   // ca se mearga in dreapta motoarele trebuie sa mearga + - - +
                velocityY - velocityX,
                velocityY + velocityX
        );
        List<Double> rotationValues = Arrays.asList(
                velocityR,  // componentele de la rotatie
                -velocityR,
                velocityR,
                -velocityR

        );

        double scaleFactor=1, tmpScaleFactor=1;

        // puterepemotor = translatevalues*scalefactor + rotationvalues
        // pt ca nu vrem ca putere motor sa intreaca 1: 1=translateValues*scalefactor+rotationvalues
        // 1-rotationvalues = translatevalues*scalefactor
        // scalefactor = (1-rotationvalues)/translatevalues
        for (int i=0; i<4; i++){//parc puterile fiecarui motor pt a ne asigura ca nu intrece 1
            if(Math.abs(translateValues.get(i) + rotationValues.get(i)) > 1)
                tmpScaleFactor = (1 - rotationValues.get(i))/translateValues.get(i);
            if(scaleFactor > tmpScaleFactor)
                scaleFactor = tmpScaleFactor;
        }
        for(int i=0; i<4; i++){
            double vit = translateValues.get(i)*scaleFactor + rotationValues.get(i);
            if(Math.abs(vit) < 0.15)//ca sa nu ii dam viteza prea mica
                vit = 0;
            if(Math.abs(vit) > 1){
                telemetry.addData("SCALE: vit mai mare decat 1 la:", i);
                //telemetry.update();
                vit=1;
            }
            powerMotors.add(vit);
        }

    }


    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
