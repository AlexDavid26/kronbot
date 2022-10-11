package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servo")
public class TestServo extends OpMode {

    Servo servo;
    Servo servo2;

    double position = 1;
    double position2 = 1;

    boolean upPressed = false;
    boolean downPressed = false;

    boolean upPressed2 = false;
    boolean downPressed2 = false;



    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo2 = hardwareMap.servo.get("servo2");
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper&&!upPressed){
            upPressed=true;
            position += 0.01;
        }
        else if(!gamepad1.right_bumper)
            upPressed=false;

        if(gamepad1.left_bumper&&!downPressed){
            downPressed=true;
            position -= 0.01;
        }
        else if(!gamepad1.left_bumper)
            downPressed=false;

        if (position > 1)
            position = 1;
        else if (position < 0)
            position = 0;

        telemetry.addData("Position:", position);

        servo.setPosition(position);



        if(gamepad1.y&&!upPressed2){
            upPressed2=true;
            position2 += 0.01;
        }
        else if(!gamepad1.y)
            upPressed2=false;

        if(gamepad1.a&&!downPressed2){
            downPressed2=true;
            position2 -= 0.01;
        }
        else if(!gamepad1.a)
            downPressed2=false;

        if (position2 > 1)
            position2 = 1;
        else if (position2 < 0)
            position2 = 0;

        servo2.setPosition(position2);

        telemetry.addData("Position 2:", position2);
        telemetry.update();
    }
}
