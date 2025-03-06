package opmode;


import static config.util.RobotConstants.specClawClose;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import config.subsystem.ServoArmSubsystem;
import config.subsystem.SpecClawSubsystem;
import config.subsystem.SpecLiftSubsystem;
import config.subsystem.RegLiftSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import static config.util.RobotConstants.*;

@TeleOp(name = "Active Observation Teleop", group = "Examples")
public class ActiveObservationTeleop extends OpMode {
    public enum DriveState {
        DRIVERCONTROL, TOPICKUP, TOSCORE, FROMSCORE, SCORING, PICKUP
    }
    PathChain scoreFromPickup, scoreToPickup;
    public DriveState driveState;
    private Follower follower;
    public SpecLiftSubsystem specLift;
    public RegLiftSubsystem regLift;
    public ServoArmSubsystem servoArm;
    public ServoArmSubsystem.ArmState armState;
    public SpecClawSubsystem specClaw;
    public SpecClawSubsystem.ClawGrabState clawState;
    public boolean specliftPIDF = true;
    public double specliftManual = 0;
    private Timer pathTimer;


    private final Pose specPickupPose = new Pose(9.5, 36.25, Math.toRadians(90));
    private final Pose scorePose = new Pose(38, 66, Math.toRadians(270));
    private final Pose startPose = scorePose;

    public void setState(DriveState driveState) {
        if (driveState == DriveState.DRIVERCONTROL) {
            follower.startTeleopDrive();
            this.driveState = DriveState.DRIVERCONTROL;
        }else if(driveState == DriveState.SCORING){
            specLift.toHang();
            this.driveState = DriveState.SCORING;
        }else if (driveState == DriveState.TOSCORE) {
            specLift.toScore();
            follower.followPath(scoreFromPickup);
            this.driveState = DriveState.TOSCORE;
        }else if (driveState == DriveState.PICKUP) {
            pathTimer.resetTimer();
            specClaw.close();
            this.driveState = DriveState.PICKUP;
        }else if (driveState == DriveState.FROMSCORE){
            specLift.toHumanPlayer();

            follower.followPath(scoreToPickup);

            this.driveState = DriveState.FROMSCORE;
        }else if (driveState == DriveState.TOPICKUP) {
            specLift.toHumanPlayer();
            specClaw.close();
            follower.followPath(follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(follower.getPose()),
                                    new Point(specPickupPose)
                            )
                    )
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), specPickupPose.getHeading())
                    .build());
            this.driveState = DriveState.TOPICKUP;
        }
    }

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        servoArm = new ServoArmSubsystem(hardwareMap, armState);
        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        specClaw = new SpecClawSubsystem(hardwareMap, clawState);
        regLift = new RegLiftSubsystem(hardwareMap, telemetry);

        Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        scoreFromPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specPickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(specPickupPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(specPickupPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), specPickupPose.getHeading())
                .build();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }


    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        setState(DriveState.DRIVERCONTROL);
        pathTimer = new Timer();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if(!specliftPIDF)
            specLift.manual(specliftManual);
        else
            specLift.updatePIDF();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Current State", driveState);

        //Triggers
        if(gamepad1.right_trigger > 0.1){
            follower.setPose(specPickupPose);
        }

        if(gamepad1.left_trigger > 0.1){
            if(driveState == DriveState.DRIVERCONTROL){
                follower.breakFollowing();
                setState(DriveState.TOPICKUP);
            }else if(driveState == DriveState.TOPICKUP && !follower.isBusy()) {
                setState(DriveState.PICKUP);
            }else if(driveState == DriveState.PICKUP && pathTimer.getElapsedTimeSeconds() > 0.5) {
                setState(DriveState.TOSCORE);
            }else if(driveState == DriveState.TOSCORE && !follower.isBusy()){
                setState(DriveState.SCORING);
                specLift.toHumanPlayer();
            }else if(driveState == DriveState.SCORING && specLift.getPos() <  1710){
                specClaw.open();
                setState(DriveState.FROMSCORE);
            }else if(driveState == DriveState.FROMSCORE && !follower.isBusy()){
                setState(DriveState.TOSCORE);
            }else if(driveState == DriveState.FROMSCORE && follower.isBusy()){
                if(follower.getCurrentTValue() >= 0.80) {
                    specClaw.close();;
                }
            }
        }else{
            if(driveState == DriveState.TOPICKUP || driveState == DriveState.TOSCORE || driveState == DriveState.FROMSCORE) {
                follower.breakFollowing();
                setState(DriveState.DRIVERCONTROL);
            }
        }

        //Bumpers
        if(gamepad1.left_bumper){
            follower.setMaxPower(0.3);
        }
        if(gamepad1.right_bumper){
            follower.setMaxPower(1);
        }



        //Reg & Spec Lift + CLaw
        if(driveState == DriveState.DRIVERCONTROL) {
            regLift.manual(-gamepad2.left_stick_y);
            specLift.manual(-gamepad2.right_stick_y);
            if (gamepad2.left_bumper) {
                gamepad2.rumble(500);
                specClaw.open();
            } else if (gamepad2.right_bumper) {
                gamepad2.rumble(500);
                specClaw.close();
            }

            //Spinning
            if (gamepad2.left_trigger > 0.1) {
                servoArm.eject();
            }
            else if (gamepad2.right_trigger > 0.1) {
                servoArm.intake();
            }
            else{
                servoArm.pause();
            }

            //Arm Pos
            if (gamepad2.dpad_left) {
                servoArm.toReturn();
            }else if (gamepad2.a) {
                servoArm.specimenGrab();
            }else if (gamepad2.x) {
                servoArm.toMiddle();
            }else if (gamepad2.dpad_right) {
                servoArm.setPos(0.695,0.59);
            } else if (gamepad2.dpad_down) {
                servoArm.setPos(0.835,0.59);
            } else if (gamepad2.y) {
                servoArm.toBasket();
            }
        }


        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}