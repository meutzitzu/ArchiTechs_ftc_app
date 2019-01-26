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

        robot.init(hardwareMap, true, telemetry);
        waitForStart();

        runtime.reset();

        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()){
            robot.liftMovement(robot.LIFT_SPEED);
        }

        robot.liftMovement(0);

        robot.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivetrainPosition(1800);

        while(robot.driveRearLeft.isBusy()
        || robot.driveFrontRight.isBusy()||
                robot.driveRearRight.isBusy() ||
                robot.driveFrontLeft.isBusy()){
            if(robot.driveRearLeft.getCurrentPosition() > 200 && robot.mechLiftLeft.getCurrentPosition() > 0){
                robot.liftMovement((-robot.LIFT_SPEED));
            }
        }

        while(robot.mechLiftLeft.getCurrentPosition() > 0)
            robot.liftMovement(-robot.LIFT_SPEED);


    }
}
