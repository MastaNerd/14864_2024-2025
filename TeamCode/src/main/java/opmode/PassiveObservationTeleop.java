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
import config.subsystem.SpecArmSubsystem;
import config.subsystem.SpecClawSubsystem;
import config.subsystem.SpecLiftSubsystem;
import config.subsystem.RegLiftSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import static config.util.RobotConstants.*;

@TeleOp(name = "Passive Observation Teleop", group = "Examples")
public class PassiveObservationTeleop extends OpMode {
    public enum DriveState {
        DRIVERCONTROL, TOPICKUP, TOSCORE, FROMSCORE, SCORING, PICKUP
    }
    PathChain scoreFromPickup, scoreToPickup;
    public DriveState driveState;
    private Follower follower;
    public SpecLiftSubsystem specLift;
    public RegLiftSubsystem regLift;
    public SpecArmSubsystem specArm;
    public SpecArmSubsystem.ArmState armState;
    public SpecClawSubsystem specClaw;
    public SpecClawSubsystem.ClawGrabState clawState;
    public boolean specliftPIDF = true;
    public double specliftManual = 0;
    private Timer pathTimer;


    private final Pose specPickupPose = new Pose(9.5, 24, Math.toRadians(90));
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
        specArm = new SpecArmSubsystem(hardwareMap, armState);
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
            if (gamepad2.dpad_left) {
                gamepad2.rumble(500);
                specClaw.open();
            } else if (gamepad2.dpad_right) {
                gamepad2.rumble(500);
                specClaw.close();
            }

            //Arm Pos
            if (gamepad2.dpad_up) {
                // Folded
                specArm.setPos(0.13,0.36,0.39);
            }
            if (gamepad2.left_bumper) {
                // Spec Pick Up
                specArm.setPos(0.205,0.76,0.79);
            }
            if (gamepad2.a) {
                // Pick Up
                specArm.setPos(0.79,0.57,0.6);
            }
            if (gamepad2.b) {
                // rotated Pick Up
                specArm.setPos(0.79,0.87,0.34);
            }
            if (gamepad2.right_trigger >= 0.2) {
                // ClawClose
                specArm.closeClaw();
            }
            if (gamepad2.left_trigger >= 0.2) {
                // ClawClose
                specArm.openClaw();
            }
            if (gamepad2.right_bumper) {
                // MidPhase
                specArm.setPos(0.205,0.36,0.39);
            }
            if (gamepad2.y) {
                specArm.setPos(0.5,0.52,0.55);
            }
            if (gamepad2.x) {
                // Entry
                specArm.setPos(0.74,0.57,0.6);
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