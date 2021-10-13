package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.drawable.GradientDrawable;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Autonomous
public class EncoderExample extends LinearOpMode {

    // Define motors
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;

    BNO055IMU imu;
    Orientation angles;

    /*Encoder Modes:
    * RUN_WITHOUT_ENCODER - sets motor power directly to the motor without adjusting the motor
    * RUN_WITH_ENCODER - uses a PID loop to check the velocity of the motor to make sure it is at the right speed
    * RUN_TO_POSITION - sets a target positions and once it reaches it holds the position
    * STOP_AND_RESET_ENCODER - reset the encoders count back to zero
    * */

    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMU.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoders();

        waitForStart();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //defined in the driveForward function below, speed is 1(100%), encoder ticks is 1000(depends on your motor), target angle is 90 degrees

        driveForward(1, 1000, 90);
    }



    //Creates a function to reset the encoders
    void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




    //Creates a function to run to certain position at a certain speed using encoders
    void driveForward(double power, double ticks, float targetAngle) throws InterruptedException{

        double leftPower;
        double rightPower;

        //sets to intended power
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        //while the robot is not at specified encoder ticks and the opmode is active, this while loop will run
        while(leftRear.getCurrentPosition() < ticks && leftFront.getCurrentPosition() < ticks && rightFront.getCurrentPosition() < ticks && rightRear.getCurrentPosition() < ticks && opModeIsActive())

            //uses the imu to get the angular orientation to find the heading angle
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //adjusts the motor powers as necessary to hold the angles as best as it can
            if(angles.firstAngle < targetAngle){
                rightPower = power + .05;
                leftPower = power - .05;
            }
            else if(angles.firstAngle > targetAngle){
                leftPower = power + .05;
                rightPower = power - .05;
            }
            else{
                leftPower = power;
                rightPower = power;
            }

            //sets motors to 0 (stops) once you break out of the while loop
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            //resets the encoders
            resetEncoders();
    }

    //The imu takes some time to update. It will overshoot because it does not have enough time to calculate the turn angle
    void turnLeft(double turnAngle, double timeouts){

        //This sleep is important so that the imu has enough time to calculate
        sleep(250);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed = .5;
        double oldDegreesLeft = turnAngle;
        double scaledSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;

        //Imu is structured from -180 to 180, not 0 to 360
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}

        //Telling us what angle we want to get to, and how many degrees we need to turn to get there
        double degreesLeft = ((Math.signum(angles.firstAngle-targetHeading)+1)/2) *(360-Math.abs(angles.firstAngle-targetHeading))+(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);

        //We want to break out of the loop if we reach a certain time (e.g 5 seconds). We don't want the turn to take forever
        resetStartTime();

            while(opModeIsActive() && time < timeouts && degreesLeft>1&& oldDegreesLeft-degreesLeft>=0){
                scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
                if(scaledSpeed>1){
                    scaledSpeed=.1;
                }

                leftFront.setPower(scaledSpeed);
                leftRear.setPower(scaledSpeed);
                rightFront.setPower(scaledSpeed);
                rightRear.setPower(scaledSpeed);

                //This checks if we have overshot the angle, and if we did, how many degrees we need to turn to get back
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                oldDegreesLeft=degreesLeft;
                degreesLeft = ((Math.signum(angles.firstAngle-targetHeading)+1)/2) *(360-Math.abs(angles.firstAngle-targetHeading))+(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
                if(Math.abs(angles.firstAngle-oldAngle)<1){
                    speed*=1.1;
                }
                oldAngle=angles.firstAngle;


        }
            //once the turn is complete, we stop the robot and reset the encoders.
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        resetEncoders();
    }
}