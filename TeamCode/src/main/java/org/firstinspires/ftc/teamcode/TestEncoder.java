package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "Test Encoder")
public class TestEncoder extends OpMode {

    DcMotor verticalLeft;


    @Override
    public void init() {
        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        telemetry.addData("Position: ", verticalLeft.getCurrentPosition());
    }
}
