package opmode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import config.subsystem.ServoArmSubsystem;
import config.subsystem.SpecLiftSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import static config.util.RobotConstants.*;

@Autonomous(name = "(use in comp)5-Specimen", group = "Examples")
public class RightAuto extends OpMode {
    public SpecLiftSubsystem specLift;
    public ServoArmSubsystem servoArm;
    public ServoArmSubsystem.ArmState armState;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private DcMotor SpecimenSlideMotor;
    private Servo SpecimenServo,LeftArmServo, RightArmServo, ClawWrist;
    private CRServo ClawSpinner;

    public boolean actionBusy, specliftPIDF = true;
    public double specliftManual = 0;


    private final Pose startPose = new Pose(7.125, 63, Math.toRadians(270));
    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(38, 66, Math.toRadians(270));
    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(35, 35, Math.toRadians(300));
    /** Lowest (First) Sample from the Spike Mark */
    private final Point pickupCPoint1 = new Point(33, 51, Point.CARTESIAN);
    /**Dropoff Pose for HP */
    private final Pose place1Pose = new Pose(25, 38, Math.toRadians(-143));
    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(31.5, 27, Math.toRadians(-60));
    /** Lowest (First) Sample from the Spike Mark */
    private final Point pickupCPoint2 = new Point(38.000, 54.000, Point.CARTESIAN);
    /**Dropoff Pose for HP */
    private final Pose place2Pose = new Pose(27, 27, Math.toRadians(-145));
    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(36, 20.5, Math.toRadians(-80));
    /** Lowest (First) Sample from the Spike Mark */
    private final Point pickupCPoint3 = new Point(58.000, 39.000, Point.CARTESIAN);
    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose place3Pose = new Pose(27, 28, Math.toRadians(-150));
    private final Point placeCPoint = new Point(43, 32.000, Point.CARTESIAN);

    private final Pose specPickupPoseMid = new Pose(15.75, 32.65, Math.toRadians(27.5));
    private final Pose specPickupPose = new Pose(9.5, 36.25, Math.toRadians(90));
    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkPose = new Pose(10, 20, Math.toRadians(90));
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain splinetoPickup1,toPickup1, toPickup2, toPickup3, grabPickup1, grabPickup2, grabPickup3, placePickup1, placePickup2, placePickup3, specPickup, scoreFromPickup, scoreToPickup, specPickupMid, toPark;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        toPickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                pickupCPoint1,
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(),0.8)
                .build();

        placePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup1Pose),
                                new Point(place1Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), place1Pose.getHeading())
                .build();

        toPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place1Pose),
                                pickupCPoint2,
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(place1Pose.getHeading(), pickup2Pose.getHeading(), 0.4)
                .build();

        placePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup2Pose),
                                new Point(place2Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), place2Pose.getHeading())
                .build();

        toPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place2Pose),
                                pickupCPoint3,
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(place2Pose.getHeading(), pickup3Pose.getHeading(), 0.4)
                .build();

        placePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup3Pose),
                                placeCPoint,
                                new Point(place3Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), place3Pose.getHeading())
                .build();

        specPickupMid = follower.pathBuilder()
                .addPath(new BezierLine(new Point(place3Pose), new Point(specPickupPoseMid)))
                .setLinearHeadingInterpolation(place3Pose.getHeading(), specPickupPoseMid.getHeading())
                .build();
        specPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specPickupPoseMid), new Point(specPickupPose)))
                .setLinearHeadingInterpolation(specPickupPoseMid.getHeading(), specPickupPose.getHeading())
                .build();
        scoreFromPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specPickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(specPickupPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(specPickupPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), specPickupPose.getHeading())
                .build();
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }
    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                specLift.setTarget(2000);
                SpecimenServo.setPosition(1);
                servoArm.toTop();
                setPathState(1);
                break;
            case 1:
                servoArm.toMiddle();
                if(!follower.isBusy()) {
                    specLift.setTarget(1700);
                    setPathState(2);
                }

                break;
            case 2:
                if(SpecimenSlideMotor.getCurrentPosition() <  1710) {
                    ClawSpinner.setPower(1);
                    SpecimenServo.setPosition(0.15);
                    specLift.setTarget(specLift.bottom);
                    follower.followPath(toPickup1, true);
                    servoArm.specimenGrab();;
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.getCurrentTValue() > 0.8){
                    follower.setMaxPower(0.5);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(placePickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(follower.getCurrentTValue() > 0.4){
                    servoArm.specimenPlace();
                }
                if(!follower.isBusy()) {
                    ClawSpinner.setPower(-1);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    /* Grab Sample */
                    servoArm.specimenGrab();;
                    follower.followPath(toPickup2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    ClawSpinner.setPower(1);
                }
                if(follower.getCurrentTValue() >  0.8){
                    follower.setMaxPower(0.5);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(placePickup2,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(follower.getCurrentTValue() > 0.4){
                    servoArm.specimenPlace();
                }
                if(!follower.isBusy()) {
                    ClawSpinner.setPower(-1);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    /* Grab Sample */
                    servoArm.specimenGrab();
                    follower.followPath(toPickup3,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    ClawSpinner.setPower(1);
                }
                if(follower.getCurrentTValue() >  0.8){
                    follower.setMaxPower(0.3);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(placePickup3,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(follower.getCurrentTValue() > 0.4){
                    servoArm.specimenPlace();
                }
                if(!follower.isBusy()) {
                    ClawSpinner.setPower(-1);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(specPickupMid,false);
                    servoArm.toReturn();
                    specLift.toHumanPlayer();
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    ClawSpinner.setPower(1);
                }
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    ClawSpinner.setPower(0);
                    follower.followPath(specPickup,true);
                    specLift.toHumanPlayer();
                    setPathState(16);
                }
                break;
            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    SpecimenServo.setPosition(specClawClose);
                    setPathState(17);
                }
                break;
            case 17:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scoreFromPickup,true);
                    specLift.setTarget(2000);
                    SpecimenServo.setPosition(1);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    specLift.setTarget(1700);
                    setPathState(19);
                }

                break;
            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(SpecimenSlideMotor.getCurrentPosition() <  1710) {
                    SpecimenServo.setPosition(0.15);
                    specLift.toHumanPlayer();
                    follower.followPath(scoreToPickup, true);
                    setPathState(20);
                }
                break;
            case 20:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getCurrentTValue() >= 0.80) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    SpecimenServo.setPosition(specClawClose);
                    setPathState(21);
                }
                break;
            case 21:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scoreFromPickup,true);
                    specLift.setTarget(2000);
                    SpecimenServo.setPosition(1);
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    specLift.setTarget(1700);
                    setPathState(23);
                }

                break;
            case 23:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(SpecimenSlideMotor.getCurrentPosition() <  1710) {
                    SpecimenServo.setPosition(0.15);
                    specLift.setTarget(specLift.bottom);
                    follower.followPath(scoreToPickup, true);
                    setPathState(24);
                }
                break;
            case 24:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getCurrentTValue() >= 0.80) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    SpecimenServo.setPosition(specClawClose);
                    setPathState(25);
                }
                break;
            case 25:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scoreFromPickup,true);
                    specLift.setTarget(2000);
                    SpecimenServo.setPosition(1);
                    setPathState(26);
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    specLift.setTarget(1700);
                    setPathState(27);
                }

                break;
            case 27:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(SpecimenSlideMotor.getCurrentPosition() <  1710) {
                    SpecimenServo.setPosition(0.15);
                    specLift.toHumanPlayer();
                    follower.followPath(scoreToPickup, true);
                    setPathState(28);
                }
                break;
            case 28:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getCurrentTValue() >= 0.80) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    SpecimenServo.setPosition(specClawClose);
                    setPathState(29);
                }
                break;
            case 29:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scoreFromPickup,true);
                    specLift.setTarget(2000);
                    SpecimenServo.setPosition(1);
                    setPathState(30);
                }
                break;
            case 30:
                if(!follower.isBusy()) {
                    specLift.setTarget(1700);
                    setPathState(31);
                }

                break;
            case 31:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(SpecimenSlideMotor.getCurrentPosition() <  1710) {
                    SpecimenServo.setPosition(0.15);
                    specLift.setTarget(specLift.bottom);
                    follower.followPath(scoreToPickup, true);
                    setPathState(32);
                }
                break;
            case 32:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

        }
    }
    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        if(!specliftPIDF)
            specLift.manual(specliftManual);
        else
            specLift.updatePIDF();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        SpecimenSlideMotor = hardwareMap.get(DcMotor.class, "Specimen Slide Motor");
        SpecimenSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SpecimenSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SpecimenServo = hardwareMap.get(Servo.class, "Specimen Servo");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        servoArm = new ServoArmSubsystem(hardwareMap, armState);
        servoArm.init();

        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        buildPaths();
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        specLift.start();
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}