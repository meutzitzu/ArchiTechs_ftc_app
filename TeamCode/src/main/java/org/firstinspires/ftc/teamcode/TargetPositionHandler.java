package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TargetPositionHandler implements Runnable {

    private DcMotor motor;
    private int targetPosition;

    TargetPositionHandler(DcMotor dcMotor){
        this.motor = dcMotor;
    }

    public void setTargetPosition(int targetPosition){
        this.targetPosition = motor.getCurrentPosition() + targetPosition;
    }

    @Override
    public void run() {

        int motorSpeed = targetPosition < motor.getCurrentPosition() ? -1 : 1;

        while(Math.abs(targetPosition - motor.getCurrentPosition()) > 10){
            motor.setPower(motorSpeed);
        }

        motor.setPower(0);
    }
}
