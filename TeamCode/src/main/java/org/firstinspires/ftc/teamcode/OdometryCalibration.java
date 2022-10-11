package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration")
public class OdometryCalibration extends LinearOpMode {
    //Drive motors
    RobotTest robot = new RobotTest();
    //DcMotor Robot.FrightMotor, right_back, left_front, left_back;
    //Odometry Wheels
    //DcMotor verticalLeft, verticalRight, horizontal;

    //IMU Sensor
    BNO055IMU imu;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "verticalL", verticalRightEncoderName = "verticalR", horizontalEncoderName = "horizontal";

    final double PIVOT_SPEED = 0.4;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 685;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;
    DcMotor orizontal;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();


        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 90 && opModeIsActive()){
            if(getZAngle() < 60) {
                robot.drive(1, -1, 1, -1, PIVOT_SPEED);
            }else{
                robot.drive(1, -1, 1,  -1, PIVOT_SPEED*3/4);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        robot.drive(0,0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.flMotor.getCurrentPosition()) + (Math.abs(robot.frMotor.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = orizontal.getCurrentPosition()/Math.toRadians(getZAngle()); // cate tickuri face pe fiecare grad

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", robot.flMotor.getCurrentPosition());
            telemetry.addData("Vertical Right Position", -robot.frMotor.getCurrentPosition());
            telemetry.addData("Horizontal Position", orizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    private void initHardwareMap(String vlEncoderName, String vrEncoderName, String hEncoderName){

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

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }
}
