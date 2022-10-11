package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous(name = "Test Range Sensor", group="Sensor")

public class TestRangeSensor extends LinearOpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void runOpMode(){
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm",rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
