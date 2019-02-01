[1mdiff --git a/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java b/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java[m
[1mindex 43469b1..1fc7835 100644[m
[1m--- a/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java[m
[1m+++ b/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java[m
[36m@@ -177,11 +177,13 @@[m [mpublic class Robot {[m
 [m
     //Motors used for lifting the robot[m
 [m
[31m-        public void liftMovement(double liftPower){[m
[31m-            liftPower = Range.clip(liftPower, -1, 1) ;[m
[32m+[m[32m        public void liftMovement(double liftPower) {[m
[32m+[m[32m            liftPower = Range.clip(liftPower, -1, 1);[m
[32m+[m[32m            if (mechLiftLeft.getCurrentPosition() > 0 && mechLiftLeft.getCurrentPosition() < MAX_LIFT_POSITION) {[m
[32m+[m[32m                mechLiftLeft.setPower(liftPower);[m
[32m+[m[32m                mechLiftRight.setPower(liftPower);[m
 [m
[31m-            mechLiftLeft.setPower(liftPower);[m
[31m-            mechLiftRight.setPower(liftPower);[m
[32m+[m[32m            }[m
         }[m
 [m
     //"PID" for motor used for arm rotation[m
