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

@Autonomous(name = "5-Sample", group = "Examples")
public class LeftAuto extends OpMode {
    public SpecLiftSubsystem specLift;
    public RegLiftSubsystem regLift;
    public ServoArmSubsystem servoArm;
    public ServoArmSubsystem.ArmState armState;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotor SpecimenSlideMotor;
    private Servo SpecimenServo, LeftArmServo, RightArmServo, ClawWrist;
    private CRServo ClawSpinner;

    public boolean actionBusy, specLiftPIDF, regLiftPIDF = true;
    public double specliftManual = 0;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.000, 105.000, Math.toRadians(90));
    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose offWallPose = new Pose(17.000, 112.000, Math.toRadians(90));
    private final Pose basketPose = new Pose(22.000, 122.000, Math.toRadians(135));
    private final Pose scorePose = new Pose(17.000, 127.000, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(36, 109.000, Math.toRadians(60));
    private final Point pickup1CPoint = new Point(35, 77, Point.CARTESIAN);
    private final Pose pickup2Pose = new Pose(36, 119.500, Math.toRadians(60));
    private final Point pickup2CPoint = new Point(42, 77, Point.CARTESIAN);
    private final Pose pickup3Pose = new Pose(43, 120.5, Math.toRadians(80));
    private final Point pickup3CPoint = new Point(54, 102, Point.CARTESIAN);
    private final Point score3CPoint = new Point(45, 116, Point.CARTESIAN);
    private final Pose midpickup4Pose = new Pose(12.000, 80.000, Math.toRadians(-90));
    private final Pose pickup4Pose = new Pose(12.000, 37.000, Math.toRadians(-90));


    private Path scorePreload;
    private PathChain offWall, score1, score2, score3, score4, score5, pickup2, pickup3, pickup4, pickup5, toBasket2, toBasket3, toBasket4, toBasket5, midPickup5, midBasket5;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        offWall = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(offWallPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), offWallPose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(offWallPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(offWallPose.getHeading(), scorePose.getHeading())
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                pickup1CPoint,
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), 0.6)
                .build();

        toBasket2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup1Pose),
                                new Point(basketPose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), basketPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(basketPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(basketPose.getHeading(), scorePose.getHeading())
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                pickup2CPoint,
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading(), 0.6)
                .build();

        toBasket3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup2Pose),
                                new Point(basketPose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), basketPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(basketPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(basketPose.getHeading(), scorePose.getHeading())
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

        toBasket4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup3Pose),
                                score3CPoint,
                                new Point(basketPose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), basketPose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(basketPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(basketPose.getHeading(), scorePose.getHeading())
                .build();

        midPickup5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(midpickup4Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), midpickup4Pose.getHeading())
                .build();

        pickup5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(midpickup4Pose),
                                new Point(pickup4Pose)
                        )
                )
                .setLinearHeadingInterpolation(midpickup4Pose.getHeading(), pickup4Pose.getHeading())
                .build();

        midBasket5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup4Pose),
                                new Point(midpickup4Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), midpickup4Pose.getHeading())
                .build();

        toBasket5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(midpickup4Pose),
                                new Point(basketPose)
                        )
                )
                .setLinearHeadingInterpolation(midpickup4Pose.getHeading(), basketPose.getHeading())
                .build();

        score5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(basketPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(basketPose.getHeading(), scorePose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(offWall);
                regLift.toHighBucket();
                servoArm.toBasket();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    servoArm.score();
                    follower.setMaxPower(0.5);
                    follower.followPath(score1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    ClawSpinner.setPower(-1);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(pickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(follower.getCurrentTValue() > 0.05){
                    regLift.toZero();
                }
                if(follower.getCurrentTValue() > 0.1){
                    servoArm.specimenGrab();
                    ClawSpinner.setPower(1);
                }
                if(follower.getCurrentTValue() > 0.6){
                    follower.setMaxPower(0.5);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    regLift.toHighBucket();
                    servoArm.toBasket();
                    follower.followPath(toBasket2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    servoArm.score();
                    follower.setMaxPower(0.5);
                    follower.followPath(score2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    ClawSpinner.setPower(-1);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(pickup3,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(follower.getCurrentTValue() > 0.05){
                    regLift.toZero();
                }
                if(follower.getCurrentTValue() > 0.1){
                    servoArm.specimenGrab();
                    ClawSpinner.setPower(1);
                }
                if(follower.getCurrentTValue() > 0.6){
                    follower.setMaxPower(0.5);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    regLift.toHighBucket();
                    servoArm.toBasket();
                    follower.followPath(toBasket3,true);
                    ClawSpinner.setPower(0);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 0.3)  {
                    servoArm.score();
                    follower.setMaxPower(0.5);
                    follower.followPath(score3,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    ClawSpinner.setPower(-1);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(pickup4,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(follower.getCurrentTValue() > 0.05){
                    regLift.toZero();
                }
                if(follower.getCurrentTValue() > 0.1){
                    servoArm.specimenGrab();
                    ClawSpinner.setPower(1);
                }
                if(follower.getCurrentTValue() > 0.6){
                    follower.setMaxPower(0.5);
                }
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    regLift.toHighBucket();
                    servoArm.toBasket();
                    follower.followPath(toBasket3,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    servoArm.score();
                    follower.setMaxPower(0.5);
                    follower.followPath(score3,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    ClawSpinner.setPower(-1);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(midPickup5,true);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(pickup5,true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(midBasket5,true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(toBasket5,true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(score5,true);
                    setPathState(20);
                }
            case 20:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

        ClawSpinner = hardwareMap.get(CRServo.class, "ClawSpinner");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        regLift = new RegLiftSubsystem(hardwareMap, telemetry);
        servoArm = new ServoArmSubsystem(hardwareMap, armState);

        servoArm.init();

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