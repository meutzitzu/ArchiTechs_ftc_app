package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="A1", group="Linear OpMode")
public class Autonom extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, false, telemetry);
        waitForStart();

        runtime.reset();

        robot.mechRotation.setTargetPosition(-400);
        robot.mechRotation.setPower(0.1);

        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()){
            robot.liftMovement(robot.LIFT_SPEED);
        }

        robot.liftMovement(0);

        robot.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivetrainPosition(600);

        while(robot.driveRearLeft.isBusy()){

//            if(robot.driveRearLeft.getCurrentPosition() < -200 && robot.mechLiftLeft.getCurrentPosition() > 0){
//                robot.liftMovement((-robot.LIFT_SPEED));
//            }
        }

        robot.driveFrontLeft.setTargetPosition(-1200);
        robot.driveFrontRight.setTargetPosition(0);
        robot.driveRearLeft.setTargetPosition(-1200);
        robot.driveRearRight.setTargetPosition(0);

        while(robot.driveFrontLeft.isBusy()){
            telemetry.addData("Status", "Forward");
            telemetry.update();
        }

//        while(robot.mechLiftLeft.getCurrentPosition() > 10)
//            robot.liftMovement(-robot.LIFT_SPEED);


    }
}
