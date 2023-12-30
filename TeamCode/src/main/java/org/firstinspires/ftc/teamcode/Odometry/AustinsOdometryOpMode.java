package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "Odometry Code Testing")
public class AustinsOdometryOpMode extends LinearOpMode {
    DcMotor FR, RR, FL, RL;
    final double TICKS_PER_ROTATION = 2000; //CHANGE THIS IF NEEDED
    //GoBilda Odometry Pod used (2000 ticks per rev, 48mm wheel diameter
    final double TICKS_PER_INCH = TICKS_PER_ROTATION / ((48 * Math.PI) / 25.4); //CHANGE THIS IF NEEDED
    //TICKS_PER_ROTATION / (omni-wheel diameter * Math.PI) / 25.4
    //This finds your wheel circumference, and turns it into inches
    final double LR_RADIUS = 5.6693;
    //5.6693 = radius of left and right dead wheel
    //MAKE SURE THESE DEAD WHEELS ARE IN LINE WITH EACH OTHER!!
    final double CENTER_TO_AUX = 4.896646;
    //4.896646 = distance from center of robot and the horizontal dead wheel
    //MAKE SURE THE HORIZONTAL ONE IS CENTERED!!

    //Create coordinate system, we want our starting position to be 0,0,0
    double RR_offset = 0;
    double FL_offset = 0;
    double FR_offset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //resetEncoder();

        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();

        goToPosition(12, 12, 0,0.25, 0.1, 1, 2);
        sleep(500);
        goToPosition(-12, -12, 0,0.25, 0.1, 1, 2);
    }


    /**
     * Calculates the distance to the new points
     *
     * @param changeX X change in distance relative to the robot
     * @param changeY Y change in distance relative to the robot
     * @return diagonal distance to the relative points
     */
    private double getDistance(double changeX, double changeY) {
        return Math.hypot(changeX, changeY);
    }

    /**
     * Calculates the angle between the new points
     *
     * @param changeX X change in distance relative to the robot
     * @param changeY Y change in distance relative to the robot
     * @return angle between the change in X and Y
     */
    private double getAngle(double changeX, double changeY) {
        return Math.atan2(changeX, changeY);

    }

    /**
     * Changes the position by a specified value and angle
     *
     * @param changeX change in X position, in inches (sideways)
     * @param changeY change in Y position, in inches (forward)
     * @param targetAngle direction to face when completed, in degrees (0 is no turn)
     * @param speed maximum lateral power
     * @param angularSpeed maximum turning power
     * @param positionTolerance range of error in ending position, in inches
     * @param angleTolerance range of error in the ending direction, in degrees
     */
    private void goToPosition(double changeX, double changeY, double targetAngle, double speed, double angularSpeed, double positionTolerance, double angleTolerance) {
        //First we need to replace some variables to read inches, not encoder ticks
        changeX *= TICKS_PER_INCH;
        changeY *= TICKS_PER_INCH;
        positionTolerance *= TICKS_PER_INCH;
        //double horizontalOffset = 0.944882 * TICKS_PER_INCH;

        //Next we need to replace some more variables to read degrees, not radians
        targetAngle = Math.toRadians(targetAngle);
        angleTolerance = Math.toRadians(targetAngle);


        // forward/drive = ((left) + (right)) / 2
        // strafe = (aux/horizontal) - k(distance from center to horizontal wheel)*theta
        // theta = ((right) - (left)) / 2r (r is the radius or the distance from the left or right dead wheel to the middle

        //Calculate the angle we need to go to in radians
        double angleToTarget = getAngle(changeX, changeY);
        double yPos = ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
        double angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
        double xPos = (((getMotorPosition("RR") - (CENTER_TO_AUX * angle))));


        //Now lets calculate a distance of how far we still have to go
        //Subtract the distance we want it to go by our current position
        double xDistance = Math.abs(changeX - xPos);
        double yDistance = Math.abs(changeY - yPos);
        double distance = getDistance(xDistance, yDistance);


        while (opModeIsActive() && (distance > positionTolerance || (angle < targetAngle - angleTolerance || angle > targetAngle + angleTolerance))) {
            //The long while specifications ensure that robot will drive if you insert a negative angle
            telemetry.addData("xDistance", xDistance / TICKS_PER_INCH);
            telemetry.addData("yDistance", yDistance / TICKS_PER_INCH);
            telemetry.addData("distance", distance / TICKS_PER_INCH);

            //calculate the distance ratio of remaining distance
            double distanceRatio = Math.abs(xDistance / yDistance);

            //Recalculate
            angleToTarget = getAngle(changeX, changeY);
            yPos = (((getMotorPosition("FR") + getMotorPosition("FL")) / 2));
            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
            xPos = ((getMotorPosition("RR") - (CENTER_TO_AUX * angle)));
            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = getDistance(xDistance, yDistance);
            distanceRatio = Math.abs(xDistance / yDistance);

            //We need to find our angle distance or how much we still have to turn
            double angleDistance = targetAngle - angle;
            //Calculate the speed for the horizontal movement
            double xSpeed = speed * Math.cos(angleToTarget) * distanceRatio;
            //Calculate the speed for the vertical movement
            double ySpeed = speed * Math.sin(angleToTarget) * distanceRatio;
            //Calculate the speed to turn the robot
            double turningSpeed = Math.signum(angleDistance) * Math.min(angularSpeed, Math.abs(angleDistance));
            //Math.signum takes the sign of angleDistance, if it is positive, the output is +1, if it is negative, the output is -1
            turningSpeed *= Math.min(1.0, Math.abs(angleDistance / (3 * angleTolerance)));
            //This is an equalizers, the robot will begin to slow down once it has reached 3 times our tolerance


            //Set the powers to each of the motors (This should be correct for all bots if you reverse your motors in init)
            double FLPower = ySpeed + turningSpeed + xSpeed;
            double FRPower = ySpeed - turningSpeed - xSpeed;
            double RRPower = ySpeed - turningSpeed + xSpeed;
            double RLPower = ySpeed + turningSpeed - xSpeed;

            //Calculate a distributor
            double frontMax = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            double backMax = Math.max(Math.abs(RLPower), Math.abs(RRPower));
            double max = Math.max(frontMax, backMax);

            //ensures that no motor can go past a value of 1
            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            //Actually set power to the motors
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            RR.setPower(RRPower);
            RL.setPower(RLPower);

            telemetry.addData("FL", FLPower);
            telemetry.addData("FR", FRPower);
            telemetry.addData("RR", RRPower);
            telemetry.addData("RL", RLPower);

            //recalculate our current position for more accurate result
            yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
            xPos = -1 * (getMotorPosition("RR") - (CENTER_TO_AUX * angle));

            telemetry.addData("xPos", xPos);
            telemetry.addData("angle", angle);
            telemetry.addData("yPos", yPos);

            //Recalculate our distance left
            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = getDistance(xDistance, yDistance);

            telemetry.update();


        }

        //Stop the motors once out of the loop
        FL.setPower(0);
        FR.setPower(0);
        RR.setPower(0);
        RL.setPower(0);
    }


    private void initialize() {
        FR = hardwareMap.dcMotor.get("FR");
        RR = hardwareMap.dcMotor.get("RR");
        FL = hardwareMap.dcMotor.get("FL");
        RL = hardwareMap.dcMotor.get("RL");

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No dead wheel

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        FR_offset = FR.getCurrentPosition();
        RR_offset = RR.getCurrentPosition();
        FL_offset = FL.getCurrentPosition();
    }

    //private void resetEncoder() {
    //FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //}

    private double getMotorPosition(String motorName) {
        switch (motorName) {
            case "FR":
                return FR.getCurrentPosition() - FR_offset;
            case "RR":
                return RR.getCurrentPosition() - RR_offset;
            case "FL":
                return FL.getCurrentPosition() - FL_offset;
            case "RL":
                return RL.getCurrentPosition();
            default:
                return 0;
        }
    }
}

