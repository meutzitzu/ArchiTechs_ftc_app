package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

public class Robot {

    /**Declare hardware variables */

        public DcMotor driveFrontLeft = null;
        public DcMotor driveFrontRight = null;
        public DcMotor driveRearLeft = null;
        public DcMotor driveRearRight = null;
        public DcMotor mechRotation = null;
        public DcMotor mechLiftLeft = null;
        public DcMotor mechLiftRight = null;
        public CRServo mechExt = null;
        public CRServo mechGrab = null;
        public Telemetry telemetry;

    /** Global constants */
        public final double MAX_CRSERVO_INPUT = 0.82;
        public final double LIFT_SPEED = 1;
        public final double ROTATION_SPEED = 0.4;
        public final int MAX_LIFT_POSITION = 25000;
        public final int MAX_ROTATION = 0;
        public final int MIN_ROTATION = -1800;
        public final int ROTATION_LENGTH = MAX_ROTATION - MIN_ROTATION;
        public double PI = 3.14159;
        public final double DRIVING_COEF = 0.6;


        //constantele mele pentru profile motioning
        public final int A = 554;
        public final int B = 738;
        public final int C = 1390;
        public final int D = 1600;
        public final int E = 2000;
        public final double alpha_one = (2*ROTATION_SPEED)/(A+B);
        public final double alpha_two = (2*alpha_one)/(E-C);

    /** Auxiliary variables */


    public void init(HardwareMap hashMap, boolean teleOp, Telemetry tele) throws InterruptedException {

        /** Initializing hardware variables */

        driveFrontLeft  = hashMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = hashMap.get(DcMotor.class, "driveFrontRight");
        driveRearLeft  = hashMap.get(DcMotor.class, "driveRearLeft");
        driveRearRight = hashMap.get(DcMotor.class, "driveRearRight");
        mechRotation = hashMap.get(DcMotor.class, "mechRotation");
        mechLiftLeft = hashMap.get(DcMotor.class, "mechLiftLeft");
        mechLiftRight = hashMap.get(DcMotor.class, "mechLiftRight");
        mechExt = hashMap.get(CRServo.class, "mechExt");
        mechGrab = hashMap.get(CRServo.class, "mechGrab");
        telemetry = tele;

        /** Reseting motors' encoders + setting mode of operation
         *  --TeleOp
         */

        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mechLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mechLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mechLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mechLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!teleOp) {
            mechRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mechRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        mechRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        /** Setting direction of hardware components
         *  --poate unele trebuie doar la TeleOp si nu la Autonom
         *  --de retinut
         */

        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotor.Direction.REVERSE);
        driveRearRight.setDirection(DcMotor.Direction.FORWARD);
        mechRotation.setDirection(DcMotor.Direction.FORWARD);
        mechLiftLeft.setDirection(DcMotor.Direction.FORWARD);
        mechLiftRight.setDirection(DcMotor.Direction.FORWARD);
        mechExt.setDirection(CRServo.Direction.FORWARD);


    }

    /** Global methods */

    //Braking (using a trigger) as a rescale
    //Almost all movement actions

        public double useBrake(double initialSpeed, double brakeFactor, boolean isCRServo){

            int speedSign = (int)(initialSpeed / abs(initialSpeed));
            double finalSpeed;
            finalSpeed = initialSpeed;

            //Clip speed for continuous servo
            if (isCRServo) {
                if(abs(initialSpeed) > MAX_CRSERVO_INPUT){
                    initialSpeed = speedSign * MAX_CRSERVO_INPUT;
                }
            }
            //Clip speed for dcmotors
            else if(!isCRServo) {
                if(abs(initialSpeed) > 1){
                    initialSpeed = speedSign * 1;
                }
            }

            //Check that the brake factor is actually braking
            if(brakeFactor <= 1 && brakeFactor >=0){
                finalSpeed = initialSpeed*brakeFactor;
            }
            else if(brakeFactor < 0){
                finalSpeed = 0;
            }
            else if (brakeFactor > 1){
                finalSpeed = initialSpeed;
            }

            return finalSpeed;
        }

    //Mecanun Driving for TeleOp
    //Can be used with robot.useBrake()

        public void mecanumMovement(double driveX, double driveY, double turn){
            double FrontLeftPower;
            double FrontRightPower;
            double RearLeftPower;
            double RearRightPower;

            //Clip value between -1 & 1
            FrontLeftPower    = Range.clip(( driveX + driveY) - turn, -1.0, 1.0) ;
            FrontRightPower   = Range.clip((-driveX + driveY) + turn, -1.0, 1.0) ;
            RearLeftPower     = Range.clip((-driveX + driveY) - turn, -1.0, 1.0) ;
            RearRightPower    = Range.clip(( driveX + driveY) + turn, -1.0, 1.0) ;

            //Set motor speed
            driveFrontLeft.setPower(FrontLeftPower);
            driveFrontRight.setPower(FrontRightPower);
            driveRearLeft.setPower(RearLeftPower);
            driveRearRight.setPower(RearRightPower);
        }


    //Motors used for lifting the robot

        public void liftMovement(double liftPower){
            liftPower = Range.clip(liftPower, -1, 1) ;

            mechLiftLeft.setPower(liftPower);
            mechLiftRight.setPower(liftPower);
        }

    //"PID" for motor used for arm rotation

        public void rotationMovement(boolean goingUp, double brakeFactor){

            double rotationSpeed = ROTATION_SPEED;
            int theta = mechRotation.getCurrentPosition();
            int error;
            boolean stop = false;
            boolean up = true;

            if(mechRotation.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
                mechRotation.setPower(0);
                mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(goingUp) {
                error = theta - MIN_ROTATION;
                if(mechRotation.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            else{
                error = theta - MIN_ROTATION;
                rotationSpeed = -rotationSpeed;
                up = false;
            }




            if(up) {

                if(error >= 0) {
                    rotationSpeed = (rotationSpeed) * Math.sin(0.23 * (tickToRad(error)) + 0.3);
                }
                else{
                    rotationSpeed = 0.3;
                }

                if(ROTATION_LENGTH - error < 10){
                    rotationSpeed = 0;
                }
            }
            else {
                if(error > (ROTATION_LENGTH *((double)2/3))){
                    if(mechRotation.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                        mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    rotationSpeed = rotationSpeed * ((double)1/error) * ((ROTATION_LENGTH * ((double)1/2)));
                }
                else if(error <= (ROTATION_LENGTH *((double)2/3)) && error > 10){

                    /** Daca vreti sa coboare bratul cu encoder, comentati linia chiar de sus
                     *      cu encoder - coboara mai controlat, dar sacadat
                     *      fara encoder - nu sacadeaza, dar poate se izbeste
                     *
                     *      Va puteti juca si cu puterea de aici un pic -> aia cu rotationSpeed = -0.05
                     *
                     * */
                    mechRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rotationSpeed = -0.05;
                }
                else {
                    rotationSpeed = -0.05;
                }
            }


            rotationSpeed = this.useBrake(rotationSpeed, brakeFactor, false);




                mechRotation.setPower(rotationSpeed);
                telemetry.addData("Error", error);

        }

        public double tickToRad(int ticks){
            return (ticks*2*PI)/1120;
        }

        public void setDrivetrainMode(DcMotor.RunMode runMode){

            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveFrontLeft.setMode(runMode);
            driveFrontRight.setMode(runMode);
            driveRearLeft.setMode(runMode);
            driveRearRight.setMode(runMode);
        }

        public void setDrivetrainPosition(int ticks){

            driveFrontLeft.setTargetPosition(-ticks);
            driveFrontRight.setTargetPosition(ticks);
            driveRearLeft.setTargetPosition(-ticks);
            driveRearRight.setTargetPosition(ticks);

            driveFrontLeft.setPower(0.3);
            driveFrontRight.setPower(0.3);
            driveRearLeft.setPower(0.3);
            driveRearRight.setPower(0.3);


        }

}
