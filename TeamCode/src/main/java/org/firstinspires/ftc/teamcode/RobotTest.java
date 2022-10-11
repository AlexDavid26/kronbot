package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotTest {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;

    Servo arm;
    Servo cupa;
    CRServo intakeSlide;
    DcMotor intake;

    DcMotor squishy;
    CRServo ruleta;

    /**
     * Set the power of the motors from wheels
     *
     * @param frontLeft  direction of the front left wheel -1 or 1
     * @param frontRight direction of the front right wheel -1 or 1
     * @param backLeft   direction of the back left wheel -1 or 1
     * @param backRight  direction of the back right wheel -1 or 1
     * @param power      the power to give to all four wheels [0,1]
     */
    void drive(double frontLeft, double frontRight, double backLeft, double backRight, double power) {
        flMotor.setPower(frontLeft * power);
        frMotor.setPower(frontRight * power);
        blMotor.setPower(backLeft * power);
        brMotor.setPower(backRight * power);
    }

    void DriveWithSpeeds(double powerbl, double powerbr, double powerfl, double powerfr){
        blMotor.setPower(powerbl);
        brMotor.setPower(powerbr);
        flMotor.setPower(powerfl);
        frMotor.setPower(powerfr);
    }


    /**
     * Stops the motors from wheels
     */
    void stopMotors() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    /**
     * Set the power of the squishy motor
     * @param power the power to give to the motor [0,1]
     */
    void setSquishyPower(double power) {
        squishy.setPower(power);
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    }
}
