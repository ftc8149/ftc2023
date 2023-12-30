package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

/**
 * Created by Sarthak on 10/4/2019.
 */
@Disabled
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor FR, RR, FL, RL;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 336.870473303; //GoBilda OdoPod (48mm wheel, 2000 tick rev)

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String frName = "FR", rrName = "RR", flName = "FL", rlName = "RL";
    String verticalLeftEncoderName = frName, verticalRightEncoderName = flName, horizontalEncoderName = rrName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(frName, rrName, flName, rlName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        goToPosition(0, 24,0.5,0,1);
        goToPosition(0, -24,0.5,0,1);
        goToPosition(24, 24, 0.5, 0, 1);
        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }
    public void goToPosition(double targetXPosition, double targetYPosition, double speed, double desiredRobotOrientation, double allowableDistanceError) {
        targetXPosition *= COUNTS_PER_INCH;
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;

        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opModeIsActive() && distance > allowableDistanceError) {

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, speed);
            double robot_movement_y_component = calculateY(robotMovementAngle, speed);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            double[] powers = rawSlide(robot_movement_x_component, robot_movement_y_component, pivotCorrection, speed);
            FR.setPower(powers[0]);
            RR.setPower(powers[1]);
            FL.setPower(powers[2]);
            RL.setPower(powers[3]);

            // Update distance after setting powers
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            // Display telemetry
            telemetry.addData("Target X", targetXPosition / COUNTS_PER_INCH);
            telemetry.addData("Target Y", targetYPosition / COUNTS_PER_INCH);
            telemetry.addData("Current X", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Current Y", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Distance to Target", distance / COUNTS_PER_INCH);
            telemetry.update();

        }

        stopMotors();
    }

    public void stopMotors() {
        FR.setPower(0);
        RR.setPower(0);
        FL.setPower(0);
        RL.setPower(0);
    }

    private void initDriveHardwareMap(String frName, String rrName, String flName, String rlName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        FR = hardwareMap.dcMotor.get(frName);
        RR = hardwareMap.dcMotor.get(rrName);
        FL = hardwareMap.dcMotor.get(flName);
        RL = hardwareMap.dcMotor.get(rlName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public static double round2D(double input) {
        input *= 100;
        input = Math.round(input);
        return input / 100;
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Takes the horizontal power, vertical power, and pivoting power and determine how much power to apply to each wheel, and normalizes the max poer
     * @param horizontal the horizontal (x vector) power
     * @param vertical the vertical (y vector) power
     * @param pivot the pivoting power
     * @param maxPower the max power the wheels can move
     */
    public double[] rawSlide(double horizontal, double vertical, double pivot, double maxPower){
        //create an array with all the speeds
        double powers[] = {vertical-horizontal+pivot, vertical+horizontal+pivot, vertical+horizontal-pivot,vertical-horizontal-pivot};

        //Only adjust speeds if the robot is moving
        if(horizontal!=0 || vertical!=0){
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for(double element:powers){
                if(Math.abs(element)>Math.abs(powers[max])){
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxCalculatedPower = Math.abs(powers[max]);

            //divide all of the speeds by the max speed to make sure that
            if(maxCalculatedPower!=0){
                powers[0]=powers[0]/maxCalculatedPower*maxPower;
                powers[1]=powers[1]/maxCalculatedPower*maxPower;
                powers[2]=powers[2]/maxCalculatedPower*maxPower;
                powers[3]=powers[3]/maxCalculatedPower*maxPower;
            }
        }
        return powers;
    }

}
