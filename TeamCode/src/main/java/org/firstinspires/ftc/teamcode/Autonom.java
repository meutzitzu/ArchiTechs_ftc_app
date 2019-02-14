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

        robot.init(hardwareMap, false, telemetry, this);
        waitForStart();

        runtime.reset();

        robot.absgyroRotation(30, "absolute");

        sleep(2000);

        robot.absgyroRotation(30, "relative");

        sleep(2000);

        robot.absgyroRotation(360, "absolute");

        stop();

        robot.mechRotation.setTargetPosition(-400);
        robot.mechRotation.setPower(0.1);

        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()){
            robot.liftMovement(robot.LIFT_SPEED);
            telemetry.addData("Status:  ", "Detaching from lander I");
        }

        robot.liftMovement(0);


        robot.setDrivetrainPosition(-600, "rotation", 0.3);

        while(robot.driveRearLeft.isBusy()) {
            telemetry.addData("Status:  ", "Detaching from lander II");
            telemetry.update();
        }

        robot.setDrivetrainPosition(300,"translation", 0.3);


        while(robot.driveFrontLeft.isBusy()){
            telemetry.addData("Status:  ", "Detaching from lander III");
            telemetry.update();
        }


        ///Here comes tensor flow stuff --- gyro would probably come in handy


        ///Placing the team marker


        ///Parking in the crater


    }
}
