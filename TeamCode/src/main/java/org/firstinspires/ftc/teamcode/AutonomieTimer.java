package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.RobotTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.StrictMath.max;

//@Autonomous(name = "Autonomie Timer")
public class AutonomieTimer extends LinearOpMode {
    RobotTest Robot = new RobotTest();
    ElapsedTime timer = new ElapsedTime();
    double acc = 0.55;
    double speed = 0.80;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();

        DriveAcc(-1, -1, -1, -1, speed, 0.5, acc, 0.02);

        Robot.arm.setPosition(0);

        timer.reset();
        while (timer.seconds() < 5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        Robot.arm.setPosition(1);

        timer.reset();
        while (timer.seconds() < 5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(1, 1, 1, 1, speed, 0.45, acc, 0.02);

        timer.reset();
        while (timer.seconds() < 2 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        Robot.drive(1,-1,1,-1,0.5);

        while(getZAngle()<85&&opModeIsActive());
        Robot.stopMotors();
        if (!opModeIsActive())
            return;

        DriveAcc(1, 1, 1, 1, speed, 1, acc, 0.02);

        Robot.intakeSlide.setPower(-1);

        timer.reset();
        while (timer.seconds() < 2.5 && opModeIsActive()) ;
        Robot.intakeSlide.setPower(0);
    }

    void DriveAcc(double bl, double br, double fl, double fb, double power, double runTime, double acc, double add) {
        timer.reset();

        while (timer.seconds() < runTime - runTime / 4) {
            acc = max(acc - add, 0);
            Robot.drive(bl, br, fl, fb, power - acc);
            if (!opModeIsActive())
                return;
        }
        while (timer.seconds() < runTime) {
            acc = Math.min(power - 0.2, acc + add);
            Robot.drive(bl, br, fl, fb, power - acc);
            if (!opModeIsActive())
                return;
        }

        Robot.drive(bl, br, fl, fb, 0);
    }

    void initHardwareMap() {
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

        Robot.flMotor = hardwareMap.dcMotor.get("fl");
        Robot.frMotor = hardwareMap.dcMotor.get("fr");
        Robot.blMotor = hardwareMap.dcMotor.get("bl");
        Robot.brMotor = hardwareMap.dcMotor.get("br");

        Robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Robot.arm = hardwareMap.servo.get("arm");
        Robot.intakeSlide = hardwareMap.crservo.get("intakeSlide");

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }
}
