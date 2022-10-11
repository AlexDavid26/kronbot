package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ManualControlSingleDriver")
public class ManualControlSingleDriver extends OpMode {
    RobotTest robot = new RobotTest();
    double EPS = 0.1;
    double ACC = 0.05;
    double currentPowerRunning;
    double currentPowerRotation;
    boolean intakeActivated = false;
    boolean intakeButtonPressed = false;

    boolean ruletaActivated = false;
    boolean ruletaButtonPressed = false;

    @Override
    public void init() {
        robot.flMotor = hardwareMap.dcMotor.get("fl");
        robot.frMotor = hardwareMap.dcMotor.get("fr");
        robot.blMotor = hardwareMap.dcMotor.get("bl");
        robot.brMotor = hardwareMap.dcMotor.get("br");

        robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.arm = hardwareMap.servo.get("arm");
        robot.cupa = hardwareMap.servo.get("cupa");
        robot.intake = hardwareMap.dcMotor.get("intake");
        robot.intakeSlide = hardwareMap.crservo.get("intakeSlide");

        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.squishy = hardwareMap.dcMotor.get("squishy");
        robot.ruleta = hardwareMap.crservo.get("ruleta");
        robot.ruleta.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void driveWheels() {
        if (gamepad1.left_trigger > EPS || gamepad1.right_trigger > EPS || Math.abs(gamepad1.right_stick_x) > EPS) {
            currentPowerRunning = 0;

            double power;

            if (gamepad1.left_trigger > EPS || gamepad1.right_stick_x < -EPS)
                power = Math.max(gamepad1.left_trigger, -gamepad1.right_stick_x);
            else
                power = Math.max(gamepad1.right_trigger, gamepad1.right_stick_x);

            if (currentPowerRotation == 0)
                currentPowerRotation = 0.1;
            else
                currentPowerRotation = Math.min(currentPowerRotation + ACC, power);

            if (gamepad1.left_trigger > EPS || gamepad1.right_stick_x < -EPS)
                robot.drive(-1, 1, -1, 1, currentPowerRotation);
            else
                robot.drive(1, -1, 1, -1, currentPowerRotation);
        } else {
            currentPowerRotation = 0;

            double xvalue = gamepad1.left_stick_x, yvalue = -gamepad1.left_stick_y;
            double angle = Math.toDegrees(Math.atan2(yvalue, xvalue));
            double power = Math.sqrt(xvalue * xvalue + yvalue * yvalue);

            if (power < EPS)
                currentPowerRunning = 0;
            else if (currentPowerRunning < power)
                currentPowerRunning = Math.min(currentPowerRunning + ACC, power);
            else
                currentPowerRunning = power;

            if (EPS < yvalue) {
                if (angle < 90) {
                    double Dial1WheelsPower = robot.map(angle, 0, 90, -1, 1);
                    if (Dial1WheelsPower < EPS)
                        Dial1WheelsPower = 0;
                    robot.drive(1, Dial1WheelsPower, Dial1WheelsPower, 1, currentPowerRunning);
                } else {
                    double Dial2WheelsPower = robot.map(angle, 90, 180, 1, -1);
                    if (Dial2WheelsPower < EPS)
                        Dial2WheelsPower = 0;
                    robot.drive(Dial2WheelsPower, 1, 1, Dial2WheelsPower, currentPowerRunning);
                }
            } else if (yvalue < -EPS) {
                if (angle < -90) {
                    double Dial3WheelsPower = robot.map(angle, -180, -90, 1, -1);
                    if (Dial3WheelsPower > -EPS)
                        Dial3WheelsPower = 0;
                    robot.drive(-1, Dial3WheelsPower, Dial3WheelsPower, -1, currentPowerRunning);
                } else {
                    double Dial4WheelsPower = robot.map(angle, -90, 0, -1, 1);
                    if (Dial4WheelsPower > -EPS)
                        Dial4WheelsPower = 0;
                    robot.drive(Dial4WheelsPower, -1, -1, Dial4WheelsPower, currentPowerRunning);
                }
            } else {
                if (-EPS < xvalue && xvalue < EPS)
                    robot.stopMotors();
                else if (xvalue > EPS)
                    robot.drive(1, -1, -1, 1, currentPowerRunning);
                else
                    robot.drive(-1, 1, 1, -1, currentPowerRunning);
            }
        }
    }

    void controlArm() {
        if (gamepad1.dpad_down)
            robot.arm.setPosition(1);
        else if (gamepad1.dpad_up)
            robot.arm.setPosition(0.47);

        if(gamepad1.dpad_left)
            robot.cupa.setPosition(0);
        else if(gamepad1.dpad_right)
            robot.cupa.setPosition(1);
    }

    void controlIntake() {
        if (gamepad1.x && !intakeButtonPressed) {
            intakeButtonPressed = true;
            intakeActivated = !intakeActivated;
        } else if (!gamepad1.x && intakeButtonPressed)
            intakeButtonPressed = false;

        if (intakeActivated)
            robot.intake.setPower(1);
        else if (gamepad1.b)
            robot.intake.setPower(-1);
        else
            robot.intake.setPower(0);
    }

    void controlIntakeSlide() {
        if (gamepad1.y)
            robot.intakeSlide.setPower(1);
        else if (gamepad1.a)
            robot.intakeSlide.setPower(-1);
        else
            robot.intakeSlide.setPower(0);
    }

    void controlSquishy() {
        if (gamepad1.right_bumper)
            robot.setSquishyPower(-0.8);
        else if (gamepad1.left_bumper)
            robot.setSquishyPower(0.8);
        else
            robot.setSquishyPower(0);
    }

    void controlRuleta(){
        if (gamepad1.back && !ruletaButtonPressed) {
            ruletaButtonPressed = true;
            ruletaActivated = !ruletaActivated;

            if(ruletaActivated)
                robot.ruleta.setPower(1);
            else
                robot.ruleta.setPower(0);
        } else if (!gamepad1.back && ruletaButtonPressed)
            ruletaButtonPressed = false;
    }

    @Override
    public void loop() {
        driveWheels();
        controlArm();
        controlIntake();
        controlIntakeSlide();
        controlSquishy();
        controlRuleta();
    }
}
