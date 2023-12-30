package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "Position Control Testing")
public class HuntersOdometryOpMode extends LinearOpMode {
    DcMotor FR, RR, FL, RL;
    final double TICKS_PER_ROTATION = 2000;
    final double TICKS_PER_INCH = TICKS_PER_ROTATION / ((48 * Math.PI) / 25.4);

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

        goToPosition(30, 1, 0,0.25, 0.1, 1, 1);
        sleep(500);
        //goToPosition(-20, -20, 0,0.25, 0, 1, 0);

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

        return Math.atan2(changeY, changeX);
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
        changeX *= TICKS_PER_INCH;
        changeY *= TICKS_PER_INCH;
        positionTolerance *= TICKS_PER_INCH;
        double horizontalOffset = 0.944882 * TICKS_PER_INCH;

        targetAngle = targetAngle * Math.PI / 180;
        angleTolerance = angleTolerance * Math.PI / 180;

        double angleToTarget = getAngle(changeX, changeY);
        double yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
        double angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * 5.6693);
        double xPos = -1 * (((getMotorPosition("RR") - (4.896646 * angle)))); // - horizontalOffset after 4.896646*angle


        double xDistance = Math.abs(Math.abs(changeX) - xPos);
        double yDistance = Math.abs(Math.abs(changeY) - yPos);
        double distance = getDistance(xDistance, yDistance);


        // forward/drive = (FR(left) + FL(right))/2
        // strafe = RR(aux/horizontal) - k(distance from center to horizontal wheel)*theta
        // theta = (FL(right) - FR(left)) / 2r (r is the radius/ the distance from the left or right dead wheel to the middle


        // while (opModeIsActive() && (distance > positionTolerance || (angle < targetAngle - angleTolerance || angle > targetAngle + angleTolerance))) {
        while (opModeIsActive() && (distance > positionTolerance ))  {
            telemetry.addData("xDistance", xDistance / TICKS_PER_INCH);
            telemetry.addData("yDistance", yDistance / TICKS_PER_INCH);
            telemetry.addData("distance", distance / TICKS_PER_INCH);

            //calculate the distance ratio of remaining distance

            angleToTarget = getAngle(changeX, changeY);
            yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * 5.6693);
            xPos = -1 * (((getMotorPosition("RR") - (4.896646 * angle)))); // - horizontalOffset after 4.896646*angle
            xDistance = Math.abs(Math.abs(changeX) - xPos);
            yDistance = Math.abs(Math.abs(changeY) - yPos);
            distance = getDistance(xDistance, yDistance);
            double distanceRatio = Math.abs(xDistance / yDistance);

            double angleDistance = targetAngle - angle;
            double xSpeed = speed * Math.cos(angleToTarget) * distanceRatio;
            double ySpeed = speed * Math.sin(angleToTarget) * distanceRatio;
            //double turningSpeed = 0; targetAngle == 0 ? angularSpeed * (targetAngle - angle) : angularSpeed * ((targetAngle - angle) / Math.abs(targetAngle));
            double turningSpeed = Math.signum(angleDistance) * Math.min(angularSpeed, Math.abs(angleDistance));
            //Math.signum takes the sign of angleDistance, if it is positive, the output is +1, if it is negative, the output is -1
            turningSpeed *= Math.min(1.0, Math.abs(angleDistance / (3 * angleTolerance)));
            //This is an equalizers, the robot will begin to slow down once it has reached 3 times our tolerance


            double FLPower = ySpeed + turningSpeed + xSpeed;
            double FRPower = ySpeed - turningSpeed - xSpeed;
            double RRPower = ySpeed - turningSpeed + xSpeed;
            double RLPower = ySpeed + turningSpeed - xSpeed;

            double frontMax = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            double backMax = Math.max(Math.abs(RLPower), Math.abs(RRPower));
            double max = Math.max(frontMax, backMax);

            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            FL.setPower(FLPower);
            FR.setPower(FRPower);
            RR.setPower(RRPower);
            RL.setPower(RLPower);

            telemetry.addData("FL", FLPower);
            telemetry.addData("FR", FRPower);
            telemetry.addData("RR", RRPower);
            telemetry.addData("RL", RLPower);

            yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * 5.6693);
            xPos = -1 * (getMotorPosition("RR") - (4.896646 * angle));

            telemetry.addData("xPos", xPos);
            telemetry.addData("angle", angle);
            telemetry.addData("yPos", yPos);

            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = getDistance(xDistance, yDistance);

            telemetry.addData("Loop", "Running");

            telemetry.update();
        }

        FL.setPower(0);
        FR.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

        sleep(500);

        //reset all variables
        angleToTarget = 0;
        yPos = 0;
        angle = 0;
        xPos = 0;
        xDistance = 0;
        yDistance = 0;
        distance = 0;
        changeX = 0;
        changeY = 0;

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
