package org.firstinspires.ftc.teamcode;

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

    /** Global constants */
        public final double MAX_CRSERVO_INPUT = 0.82;
        public final double LIFT_SPEED = 0.5;
        public final double ROTATION_SPEED = 0.3;
        public final double MAX_ROTATION = 2000;
        public final double MIN_ROTATION = 0;
        public final double ROTATION_LENGTH = MAX_ROTATION - MIN_ROTATION;

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

        mechRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mechRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
            mechLiftRight.setPower(-liftPower);
        }

    //"PID" for motor used for arm rotation

        public void rotationMovement(boolean goingUp){

            double rotationSpeed = ROTATION_SPEED;
            double theta = mechRotation.getCurrentPosition();
            double error = 0;

            if(goingUp) {
                error = MAX_ROTATION - theta;
            }
            else{
                error = theta - MIN_ROTATION;
                rotationSpeed = -rotationSpeed;
            }

                /* still testing
                    if(theta < A){
                        rotationSpeed = alpha_one*theta + 0.1;
                    }
                    else if(theta < B){
                        rotationSpeed = ((alpha_one/(2*A-2*B))*theta*theta - ((alpha_one*B)/A-B)*theta + MAX_ROTATION + ((alpha_one*B*B)/(A-B)));
                    }
                    else if(theta < C){
                        rotationSpeed = MAX_ROTATION;
                    }
                    else
                        rotationSpeed = 0;

                else if( theta >= E-10 )
                {
                    rotationSpeed = 0;
                } */
                // THIS WORKS WELL ENOUGH
            if(error > ROTATION_LENGTH*(5/6)){
                rotationSpeed = rotationSpeed * (1 / error) * ((ROTATION_LENGTH / 6)) ;
            }
            else if(error > ROTATION_LENGTH/3){

            }
            else if(error <= ROTATION_LENGTH/3 && error > 10){
                rotationSpeed = rotationSpeed * error * (1/(ROTATION_LENGTH / 3));
            }
            else {
                rotationSpeed = 0;
            }



                mechRotation.setPower(rotationSpeed);

        }
}
