package opmode;


import static config.util.RobotConstants.specClawClose;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import config.subsystem.ServoArmSubsystem;
import config.subsystem.SpecClawSubsystem;
import config.subsystem.SpecLiftSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import static config.util.RobotConstants.*;

@TeleOp(name = "Observation Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {
    public enum DriveState {
        DRIVERCONTROL, TOPICKUP, TOSCORE, FROMSCORE, SCORING
    }
    PathChain scoreFromPickup, scoreToPickup;
    public DriveState driveState;
    private Follower follower;
    private Servo LeftArmServo, RightArmServo, ClawWrist;
    public SpecLiftSubsystem specLift;
    public ServoArmSubsystem servoArm;
    public ServoArmSubsystem.ArmState armState;
    public SpecClawSubsystem specClaw;
    public SpecClawSubsystem.ClawGrabState clawState;

    private final Pose startPose = new Pose(7, 63, Math.toRadians(270));
    private final Pose specPickupPose = new Pose(9.5, 36.25, Math.toRadians(90));
    private final Pose scorePose = new Pose(38, 66, Math.toRadians(270));


    public void setState(DriveState driveState) {
        if (driveState == DriveState.DRIVERCONTROL) {
            follower.startTeleopDrive();
            this.driveState = DriveState.DRIVERCONTROL;
        }else if(driveState == DriveState.SCORING){
            specLift.toHang();
            this.driveState = DriveState.SCORING;
        }else if (driveState == DriveState.TOSCORE){
            specLift.toScore();
            follower.followPath(scoreToPickup);
            this.driveState = DriveState.TOSCORE;
        }else if (driveState == DriveState.FROMSCORE){
            specLift.toHumanPlayer();

            follower.followPath(scoreFromPickup);

            if(follower.getCurrentTValue() >= 0.80) {
                specClaw.close();
            }

            this.driveState = DriveState.FROMSCORE;
        }else if (driveState == DriveState.TOPICKUP) {
            specLift.toHumanPlayer();

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
        LeftArmServo = hardwareMap.get(Servo.class, "LeftArmServo");
        ClawWrist = hardwareMap.get(Servo.class, "ClawWrist");
        RightArmServo = hardwareMap.get(Servo.class, "RightArmServo");

        servoArm = new ServoArmSubsystem(hardwareMap, armState);
        servoArm.init();
        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        specClaw = new SpecClawSubsystem(hardwareMap, clawState);

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
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        if(gamepad1.left_bumper){
            if(driveState == DriveState.DRIVERCONTROL){
                follower.breakFollowing();
                setState(DriveState.TOPICKUP);
            }else if(driveState == DriveState.TOPICKUP && !follower.isBusy()){
                setState(DriveState.TOSCORE);
            }else if(driveState == DriveState.TOSCORE && !follower.isBusy()){
                setState(DriveState.SCORING);
                specLift.toHumanPlayer();
            }else if(driveState == DriveState.SCORING && specLift.getPos() <  1710){
                specClaw.open();
                setState(DriveState.FROMSCORE);
            }else if(driveState == DriveState.FROMSCORE && !follower.isBusy()){
                setState(DriveState.TOSCORE);
            }
        }else{
            if(driveState == DriveState.TOPICKUP || driveState == DriveState.TOSCORE || driveState == DriveState.FROMSCORE) {
                follower.breakFollowing();
                setState(DriveState.DRIVERCONTROL);
            }
        }

        if (gamepad2.left_trigger > 0.1) {
            servoArm.eject();
        }
        else if (gamepad2.right_trigger > 0.1) {
            servoArm.intake();
        }
        else if (gamepad2.dpad_left) {
            ClawWrist.setPosition(0.88);
            LeftArmServo.setPosition(0.195);
            RightArmServo.setPosition(0.15);
        } else if (gamepad2.a) {
            ClawWrist.setPosition(0.59);
            LeftArmServo.setPosition(0.855);
            RightArmServo.setPosition(0.81);
        } else if (gamepad2.x) {
            ClawWrist.setPosition(0.59);
            LeftArmServo.setPosition(0.795);
            RightArmServo.setPosition(0.75);
        } else if (gamepad2.dpad_right) {
            ClawWrist.setPosition(0.59);
            LeftArmServo.setPosition(0.695);
            RightArmServo.setPosition(0.65);
        } else if (gamepad2.dpad_down) {
            ClawWrist.setPosition(0.59);
            LeftArmServo.setPosition(0.835);
            RightArmServo.setPosition(0.79);
        } else if (gamepad2.y) {
            ClawWrist.setPosition(0.8);
            LeftArmServo.setPosition(0.595);
            RightArmServo.setPosition(0.55);
        }
        if (gamepad2.dpad_up) {
            ClawWrist.setPosition(0.3);
            LeftArmServo.setPosition(0.235);
            RightArmServo.setPosition(0.19);
        }

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}