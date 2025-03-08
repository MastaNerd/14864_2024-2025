package opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import config.subsystem.SpecArmSubsystem;
import config.subsystem.SpecClawSubsystem;
import config.subsystem.SpecLiftSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import java.lang.Math;

@Autonomous(name = "Passive 5-Specimen", group = "Examples")
public class PassiveRightAuto extends OpMode {
    public SpecLiftSubsystem specLift;
    public SpecClawSubsystem specClaw;
    public SpecClawSubsystem.ClawGrabState clawState;
    public SpecArmSubsystem specArm;
    public SpecArmSubsystem.ArmState armState;
    private Follower follower;
    private Timer pathTimer,
            opmodeTimer;
    private int pathState;
    public boolean actionBusy, specliftPIDF = true;
    public double specliftManual = 0;


    private final Pose startPose = new Pose(7, 63, Math.toRadians(270));
    private final Pose scorePose = new Pose(38, 66, Math.toRadians(270));
    private final Pose pickup1Pose = new Pose(32, 36, Math.toRadians(315));
    private final Pose place1Pose = new Pose(26, 31, Math.toRadians(225));
    private final Pose pickup2Pose = new Pose(30, 27, Math.toRadians(315));
    private final Pose place2Pose = new Pose(22, 31, Math.toRadians(225));
    private final Pose pickup3Pose = new Pose(31, 18, Math.toRadians(315));
    private final Point pickup3CP = new Point(25.5, 28.8, Point.CARTESIAN);
    private final Pose place3Pose = new Pose(22, 31, Math.toRadians(225));
    private final Pose specPickupPoseMid = new Pose(17.75, 24, Math.toRadians(27.5));
    private final Pose specPickupPose = new Pose(8.5, 36.125, Math.toRadians(90));
    private final Pose parkPose = new Pose(13, 30, Math.toRadians(225));
    private Path scorePreload;
    private PathChain splinetoPickup1,toPickup1, toPickup2, toPickup3, grabPickup1, grabPickup2, grabPickup3, placePickup1, placePickup2, placePickup3, specPickup, scoreFromPickup, scoreToPickup, specPickupMid, toPark;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        toPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        placePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(place1Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), place1Pose.getHeading())
                .build();

        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(place1Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(place1Pose.getHeading(), pickup2Pose.getHeading(),0.5)
                .build();

        placePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(place2Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), place2Pose.getHeading())
                .build();

        toPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place2Pose),
                                pickup3CP,
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(place2Pose.getHeading(), pickup3Pose.getHeading(), 0.4)
                .build();


        placePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup3Pose),
                                pickup3CP,
                                new Point(place3Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), place3Pose.getHeading(), 0.4)
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
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading(), .25)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                specLift.setTarget(2000);
                specClaw.close();
                specArm.setPos(0.5,0.52,0.55);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    specLift.setTarget(1700);
                    specArm.setPos(0.74,0.57,0.6);
                    setPathState(2);
                }

                break;
            case 2:
                if(specLift.getPos() <  1710) {
                    specClaw.open();
                    specLift.setTarget(specLift.bottom);
                    follower.followPath(toPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    specArm.setPos(0.79,0.44,0.727);
                    specArm.closeClaw();
                    setPathState(4);
                }

                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    specArm.setPos(0.74,0.44,0.727);
                    follower.followPath(placePickup1);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    specArm.openClaw();
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(toPickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    specArm.setPos(0.79,0.44,0.727);
                    specArm.closeClaw();
                    setPathState(8);
                }

                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    specArm.setPos(0.74,0.44,0.727);
                    follower.followPath(placePickup2);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    specArm.openClaw();
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(toPickup3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    specArm.setPos(0.79,0.44,0.727);
                    specArm.closeClaw();
                    setPathState(12);
                }

                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    specArm.setPos(0.74,0.44,0.727);
                    follower.followPath(placePickup3);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    specArm.openClaw();
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(specPickupMid,false);
                    specArm.setPos(0.5,0.52,0.55);
                    specLift.toHumanPlayer();
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(specPickup,true);
                    specArm.setPos(0.205,0.36,0.39);
                    specLift.toHumanPlayer();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    specClaw.close();
                    setPathState(17);
                }
                break;
            case 17:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    specArm.setPos(0.13,0.36,0.39);
                    follower.followPath(scoreFromPickup,true);
                    specLift.setTarget(2000);
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
                if(specLift.getPos() <  1710) {
                    specClaw.open();
                    specLift.toHumanPlayer();
                    follower.followPath(scoreToPickup, true);
                    setPathState(20);
                }
                break;
            case 20:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getCurrentTValue() >= 0.80) {
                    /* Grab Sample */
                    specClaw.close();
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
                    specClaw.close();
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
                if(specLift.getPos() <  1710) {
                    specClaw.open();
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
                    specClaw.close();
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
                    specClaw.close();
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
                if(specLift.getPos() <  1710) {
                    specClaw.open();
                    specLift.toHumanPlayer();
                    follower.followPath(scoreToPickup, true);
                    setPathState(28);
                }
                break;
            case 28:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getCurrentTValue() >= 0.80) {
                    /* Grab Sample */
                    specClaw.close();
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
                    specClaw.close();
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
                if(specLift.getPos() <  1710) {
                    specClaw.open();
                    specLift.setTarget(specLift.bottom);
                    follower.followPath(toPark);
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        if(!specliftPIDF)
            specLift.manual(specliftManual);
        else
            specLift.updatePIDF();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        follower.telemetryDebug(telemetry);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        specClaw = new SpecClawSubsystem(hardwareMap, clawState);

        specArm = new SpecArmSubsystem(hardwareMap, armState);
        specArm.init();

        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        specLift.start();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}