package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mihnea", group="Linear Opmode")
public class CRServoh extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private CRServo mechExt = null;

    double servo = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mechExt = hardwareMap.get(CRServo.class, "mechExt");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(gamepad1.dpad_up)
                servo = 0.9;
            else if(gamepad1.dpad_down)
                servo = -0.9;
            else if(!gamepad1.dpad_up && !gamepad1.dpad_down)
                servo = 0;

            servo = servo + gamepad1.left_stick_y;

            mechExt.setPower(servo);

            telemetry.addData("Value of Dpad_Up", gamepad1.dpad_up);
            telemetry.addData("Value of Dpad_Down", gamepad1.dpad_down);
            telemetry.addData("Servo speed", mechExt.getPower());
            telemetry.update();
        }

    }
}
