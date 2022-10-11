package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;


/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
//@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //Odometry encoder wheels
    //DcMotor verticalRight, verticalLeft, horizontal;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 700;

    RobotTest Robot = new RobotTest();
    DcMotor orizontal;
    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "verticalL", verticalRightEncoderName = "verticalR", horizontalEncoderName = "horizontal";


    @Override
    public void runOpMode() throws InterruptedException {

        //Assign the hardware map to the odometry wheels
        /* = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        /* verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        */
        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        /*verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        initHardwareMap();

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(Robot.flMotor, Robot.frMotor, orizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        /** rev la alea cu minus
         * globalPositionUpdate.reverseLeftEncoder()
         * globalPositionUpdate.reverseRightEncoder()
         * globalPositionUpdate.reverseNormalEncoder()
         *
         * */
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", Robot.flMotor.getCurrentPosition()); /// daca astea sunt negative dam reverse :)
            telemetry.addData("Vertical right encoder position", Robot.frMotor.getCurrentPosition()); /// daca astea sunt negative dam reverse :)
            telemetry.addData("horizontal left encoder position", orizontal.getCurrentPosition()); /// daca astea sunt negative dam reverse :)

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }

    void initHardwareMap()
    {

        Robot.flMotor = hardwareMap.dcMotor.get("fl");
        Robot.frMotor = hardwareMap.dcMotor.get("fr");
        Robot.blMotor = hardwareMap.dcMotor.get("bl");
        Robot.brMotor = hardwareMap.dcMotor.get("br");
        Robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        orizontal = hardwareMap.dcMotor.get("orizontal");
        orizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        orizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        Robot.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();


    }
}
