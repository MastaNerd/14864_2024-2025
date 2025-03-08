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

import config.subsystem.RegLiftSubsystem;
import config.subsystem.SpecArmSubsystem;
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

@Autonomous(name = "ACTUAL Passive 5-Sample", group = "Examples")
public class CoolIdeaPassiveLeftAuto extends OpMode {
    public SpecLiftSubsystem specLift;
    public RegLiftSubsystem regLift;
    public SpecArmSubsystem specArm;
    public SpecArmSubsystem.ArmState armState;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotor SpecimenSlideMotor;
    private Servo SpecimenServo, LeftArmServo, RightArmServo, ClawWrist;
    private CRServo ClawSpinner;

    public boolean actionBusy, specLiftPIDF, regLiftPIDF = true;
    public double specliftManual = 0;

    private final Pose startPose = new Pose(7.000, 105.000, Math.toRadians(90));
    private final Pose basketPose = new Pose(22.000, 122.000, Math.toRadians(135));
    private final Pose scorePose = new Pose(19.000, 125.000, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(32, 107, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(32, 117, Math.toRadians(45));
    private final Pose pickup3Pose = new Pose(32, 129, Math.toRadians(45));
    private final Point pickup3CPoint = new Point(23.926153846153845, 85.29230769230769, Point.CARTESIAN);
    private final Pose midSubPickupPose = new Pose(12.000, 80.000, Math.toRadians(-90));
    private final Pose subPickupPose = new Pose(12.000, 80.000, Math.toRadians(-90));


    private Path scorePreload;
    private PathChain offWall, score1, score2, score3, score4, score5, pickup2, pickup3, pickup4, midSubPickup, subPickup, midSubScore, subScore;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {


        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.9)
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), 0.6)
                .build();

        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup1Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading(), 0.6)
                .build();


        score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup2Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                pickup3CPoint,
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(), 0.6)
                .build();


        score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup3Pose),
                                pickup3CPoint,
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        midSubPickup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(midSubPickupPose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), midSubPickupPose.getHeading())
                .build();

        subPickup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(midSubPickupPose),
                                new Point(subPickupPose)
                        )
                )
                .setLinearHeadingInterpolation(midSubPickupPose.getHeading(), subPickupPose.getHeading())
                .build();

        midSubScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(subPickupPose),
                                new Point(midSubPickupPose)
                        )
                )
                .setLinearHeadingInterpolation(subPickupPose.getHeading(), midSubPickupPose.getHeading())
                .build();

        subScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(midSubPickupPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(midSubPickupPose.getHeading(), scorePose.getHeading())
                .build();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(score1);
                specArm.setPos(0.4,0.52,0.55);
                regLift.toHighBucket();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    specArm.setPos(0.5,0.52,0.55);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    specArm.openClaw();
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.setMaxPower(1);
                    follower.followPath(pickup2);
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.getCurrentTValue() > 0.4){
                    specArm.setPos(0.79,0.7,0.47);
                }
                if(follower.getCurrentTValue() > 0.2){
                    regLift.toZero();
                }

                if(!follower.isBusy()) {
                    specArm.closeClaw();
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    regLift.toHighBucket();
                    specArm.setPos(0.4,0.52,0.55);
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    specArm.setPos(0.5,0.52,0.55);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    specArm.openClaw();
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.setMaxPower(1);
                    follower.followPath(pickup3);
                    setPathState(7);
                }
                break;
            case 7:
                if(follower.getCurrentTValue() > 0.4){
                    specArm.setPos(0.79,0.7,0.47);
                }
                if(follower.getCurrentTValue() > 0.2){
                    regLift.toZero();
                }

                if(!follower.isBusy()) {
                    specArm.closeClaw();
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    regLift.toHighBucket();
                    specArm.setPos(0.4,0.52,0.55);
                    follower.followPath(score3);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    specArm.setPos(0.5,0.52,0.55);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    specArm.openClaw();
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.setMaxPower(1);
                    follower.followPath(pickup4);
                    setPathState(11);
                }
                break;
            case 11:
                if(follower.getCurrentTValue() > 0.6){
                    specArm.setPos(0.74,0.57,0.6);
                }
                if(follower.getCurrentTValue() > 0.2){
                    regLift.toZero();
                }

                if(!follower.isBusy()) {
                    specArm.setPos(0.79,0.7,0.47);
                    specArm.closeClaw();
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    regLift.toHighBucket();
                    specArm.setPos(0.4,0.52,0.55);
                    follower.followPath(score4);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    specArm.setPos(0.5,0.52,0.55);
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    specArm.openClaw();
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.setMaxPower(1);
                    follower.followPath(midSubPickup, false);
                    setPathState(15);
                }
                break;
            case 15:
                if(follower.getCurrentTValue() > 0.6){
                    specArm.setPos(0.74,0.57,0.6);
                }
                if(follower.getCurrentTValue() > 0.2){
                    regLift.toZero();
                }
                if(!follower.isBusy()) {
                    follower.followPath(subPickup,false);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    specArm.setPos(0.79,0.7,0.47);
                    specArm.closeClaw();
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    follower.followPath(midSubScore, false);
                    setPathState(18);
                }
                break;
            case 18:
                if(follower.getCurrentTValue() > 0.2){
                    regLift.toHighBucket();
                }

                if(follower.getCurrentTValue() > 0.6){
                    specArm.setPos(0.4,0.52,0.55);
                }

                if(!follower.isBusy()) {
                    follower.followPath(subScore, false);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    specArm.setPos(0.5,0.52,0.55);
                    setPathState(20);
                }
                break;
            case 20:
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    specArm.openClaw();
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.5){
                    specArm.setPos(0.205,0.36,0.39);
                }
                if(pathTimer.getElapsedTimeSeconds() > 3){
                    regLift.toZero();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4.5){
                    setPathState(21);
                }
                break;
            case 21:
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

        if(!specLiftPIDF)
            specLift.manual(specliftManual);
        else
            specLift.updatePIDF();

        if(!regLiftPIDF)
            regLift.manual(specliftManual);
        else
            regLift.updatePIDF();

        boolean isStopped = gamepad1.left_bumper;

        if (isStopped){
            follower.breakFollowing();
            setPathState(-1);
        }
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        regLift = new RegLiftSubsystem(hardwareMap, telemetry);
        specArm = new SpecArmSubsystem(hardwareMap, armState);

        specArm.closeClaw();
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