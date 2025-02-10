package config.runmodes;

import static config.util.FieldConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import config.subsystem.ServoArmSubsystem;
import config.subsystem.SpecClawSubsystem;
import config.subsystem.SpecLiftSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


public class Auto {

    private RobotStart startLocation;

    public SpecLiftSubsystem specLift;
    public ServoArmSubsystem servoArm;
    public SpecClawSubsystem specClaw;
    public ServoArmSubsystem.ArmState armState;
    public SpecClawSubsystem.ClawGrabState grabState;


    public Follower follower;
    public Telemetry telemetry;

    public boolean actionBusy, specliftPIDF = true;
    public double specliftManual = 0;

    public Timer transferTimer = new Timer(), bucketTimer = new Timer(), chamberTimer = new Timer(), intakeTimer = new Timer(), parkTimer = new Timer(), specimenTimer = new Timer(), chamberTimer2 = new Timer();
    public int transferState = -1, bucketState = -1, chamberState = -1, intakeState = -1, parkState = -1, specimenState = -1;

    public Path element1, score1, element2, score2, element3, score3, scorePreload;
    public PathChain preload, to1, to2, to3, place1, place2, place3, grab1, grab2, grab3, scoreFromPickup, scoreToPickup, specPickup, park;
    public Pose startPose, preloadPose, scorePose, sample1Pose, sample1ControlPose, sample2Pose, sample2ControlPose, sample3Pose, sample3ControlPose, sampleScorePose, parkControlPose, parkPose, grab1Pose, specimen1Pose, grab2Pose, specimen2Pose, grab3Pose, specimen3Pose, grab4Pose, specimen4Pose, specimenSetPose, to1Pose, to2Pose, to3Pose, place1Pose, place2Pose, place3Pose, specPickupPose;

    public Auto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
        specLift = new SpecLiftSubsystem(hardwareMap, telemetry);
        servoArm = new ServoArmSubsystem(hardwareMap, armState);
        specClaw = new SpecClawSubsystem(hardwareMap, grabState);

        this.follower = follower;
        this.telemetry = telemetry;

        startLocation = isBlue ? (isBucket ? RobotStart.BLUE_BUCKET : RobotStart.BLUE_OBSERVATION) : (isBucket ? RobotStart.RED_BUCKET : RobotStart.RED_OBSERVATION);

        createPoses();
        buildPaths();

        init();
    }

    public void init() {
        specLift.init();
        telemetryUpdate();

        follower.setStartingPose(startPose);
    }

    public void start() {
        specLift.start();

        follower.setStartingPose(startPose);
    }

    public void update() {
        follower.update();

        if(!specliftPIDF)
            specLift.manual(specliftManual);
        else
            specLift.updatePIDF();

        transfer();
        bucket();
        chamber();
        park();
        specimen();
        telemetryUpdate();
    }

    public void createPoses() { //Able to be cut
        switch (startLocation) {
            case BLUE_BUCKET:
                startPose = blueBucketStartPose;
                preloadPose = blueBucketPreloadPose;
                sample1ControlPose = blueBucketLeftSampleControlPose;
                sample1Pose = blueBucketLeftSamplePose;
                sample2ControlPose = blueBucketMidSampleControlPose;
                sample2Pose = blueBucketMidSamplePose;
                sample3ControlPose = blueBucketRightSampleControlPose;
                sample3Pose = blueBucketRightSamplePose;
                sampleScorePose = blueBucketScorePose;
                parkControlPose = blueBucketParkControlPose;
                parkPose = blueBucketParkPose;
                break;

            case BLUE_OBSERVATION:
                startPose = blueObservationStartPose;
                scorePose = blueObservationPreloadPose;
                to1Pose = blueObservationPickupLeftPose;
                to2Pose = blueObservationPickupMidPose;
                to3Pose = blueObservationPickupRightPose;
                grab1Pose = blueObservationPickupLeftPoseForward;
                grab2Pose = blueObservationPickupMidPoseForward;
                grab3Pose = blueObservationPickupRightPoseForward;
                place1Pose = blueObservationDropoffLeftPose;
                place2Pose = blueObservationDropoffMidPose;
                place3Pose = blueObservationSpecPickupPose;
                specPickupPose = blueObservationSpecPickupPose;


                parkPose = blueObservationParkPose;
                break;

            case RED_BUCKET:
                startPose = redBucketStartPose;
                //parkPose = redBucketPark;
                break;

            case RED_OBSERVATION:
                startPose = redObservationStartPose;
                //parkPose = redObservationPark;
                break;
        }

        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        if((startLocation == RobotStart.BLUE_BUCKET) || (startLocation == RobotStart.RED_BUCKET)) {
            preload = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();

            element1 = new Path(new BezierCurve(new Point(scorePose), new Point(sample1ControlPose), new Point(sample1Pose)));
            element1.setLinearHeadingInterpolation(scorePose.getHeading(), sample1Pose.getHeading());

            score1 = new Path(new BezierLine(new Point(sample1Pose), new Point(sampleScorePose)));
            score1.setLinearHeadingInterpolation(sample1Pose.getHeading(), sampleScorePose.getHeading());

            element2 = new Path(new BezierCurve(new Point(sampleScorePose), new Point(sample2ControlPose), new Point(sample2Pose)));
            element2.setLinearHeadingInterpolation(sampleScorePose.getHeading(), sample2Pose.getHeading());

            score2 = new Path(new BezierLine(new Point(sample2Pose), new Point(sampleScorePose)));
            score2.setLinearHeadingInterpolation(sample2Pose.getHeading(), sampleScorePose.getHeading());

            element3 = new Path(new BezierCurve(new Point(sampleScorePose), new Point(sample3ControlPose), new Point(sample3Pose)));
            element3.setLinearHeadingInterpolation(sampleScorePose.getHeading(), sample3Pose.getHeading());

            score3 = new Path(new BezierLine(new Point(sample3Pose), new Point(sampleScorePose)));
            score3.setLinearHeadingInterpolation(sample3Pose.getHeading(), sampleScorePose.getHeading());

            park = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(sampleScorePose), new Point(parkControlPose), new Point(parkPose)))
                    .setLinearHeadingInterpolation(sampleScorePose.getHeading(), parkPose.getHeading())
                    .build();
        }

        if (startLocation == RobotStart.BLUE_OBSERVATION || startLocation == RobotStart.RED_OBSERVATION) {
            scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

            to1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose), new Point(to1Pose)))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), to1Pose.getHeading())
                    .build();

            grab1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(to1Pose), new Point(grab1Pose)))
                    .setLinearHeadingInterpolation(to1Pose.getHeading(), grab1Pose.getHeading())
                    .build();

            place1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(grab1Pose), new Point(place1Pose)))
                    .setLinearHeadingInterpolation(grab1Pose.getHeading(), place1Pose.getHeading())
                    .build();

            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            to2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(place1Pose), new Point(to2Pose)))
                    .setLinearHeadingInterpolation(place1Pose.getHeading(), to2Pose.getHeading())
                    .build();

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grab2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(to2Pose), new Point(grab2Pose)))
                    .setLinearHeadingInterpolation(to2Pose.getHeading(), grab2Pose.getHeading())
                    .build();

            /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            place2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(grab2Pose), new Point(place2Pose)))
                    .setLinearHeadingInterpolation(grab2Pose.getHeading(), place2Pose.getHeading())
                    .build();

            /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            to3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(place2Pose), new Point(to3Pose)))
                    .setLinearHeadingInterpolation(place2Pose.getHeading(), to3Pose.getHeading())
                    .build();

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grab3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(to3Pose), new Point(grab3Pose)))
                    .setLinearHeadingInterpolation(to3Pose.getHeading(), grab3Pose.getHeading())
                    .build();

            place3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(grab3Pose), new Point(place3Pose)))
                    .setLinearHeadingInterpolation(grab3Pose.getHeading(), place3Pose.getHeading())
                    .build();

            specPickup = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(place3Pose), new Point(specPickupPose)))
                    .setLinearHeadingInterpolation(place3Pose.getHeading(), specPickupPose.getHeading())
                    .build();

            scoreFromPickup = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(specPickupPose), new Point(scorePose)))
                    .setLinearHeadingInterpolation(specPickupPose.getHeading(), scorePose.getHeading())
                    .build();

            scoreToPickup = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose), new Point(specPickupPose)))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), specPickupPose.getHeading())
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                    .build();

        }
    }

    public void transfer() {
        switch (transferState) {
            case 1:
                actionBusy = true;
                specLift.toTransfer();
                transferTimer.resetTimer();
                setTransferState(2);
                break;
            case 2:
                if (transferTimer.getElapsedTimeSeconds() > 1.5) {
                    transferTimer.resetTimer();
                    setTransferState(3);
                }
                break;
            case 3:
                if (transferTimer.getElapsedTimeSeconds() > 1) {
                    specLift.toZero();
                    transferTimer.resetTimer();
                    setTransferState(4);
                }
                break;
            case 4:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    actionBusy = false;
                    setTransferState(-1);
                }
                break;
        }
    }

    public void setTransferState(int x) {
        transferState = x;
    }

    public void startTransfer() {
        if (actionNotBusy()) {
            setTransferState(1);
        }
    }

    public void bucket() {
        switch (bucketState) {
            case 1:
                actionBusy = true;
                bucketTimer.resetTimer();
                setBucketState(2);
                break;
            case 2:
                if (bucketTimer.getElapsedTimeSeconds() > 0.5) {
                    bucketTimer.resetTimer();
                    setBucketState(3);
                }
                break;
            case 3:
                if (bucketTimer.getElapsedTimeSeconds() > 1) {
                    actionBusy = false;
                    setBucketState(-1);
                }

        }
    }

    public void setBucketState(int x) {
        bucketState = x;
    }

    public void startBucket() {
        if (actionNotBusy()) {
            setBucketState(1);
        }
    }

    public void chamber() {
        switch (chamberState) {
            case 1:
                actionBusy = true;
                chamberTimer.resetTimer();
                setChamberState(2);
                break;
            case 2:
                if ((follower.getPose().getX() >= specimen1Pose.getX() - 0.5)) {
                    chamberTimer.resetTimer();
                    setChamberState(3);
                }
                break;
            case 3:
                if(chamberTimer.getElapsedTimeSeconds() > 0.25) {
                    actionBusy = false;
                    setChamberState(-1);
                }
                break;
        }
    }

    public void setChamberState(int x) {
        chamberState = x;
    }

    public void startChamber() {
        if(actionNotBusy()) {
            setChamberState(1);
        }
    }

    public void specimen() {
        switch (specimenState) {
            case 1:
                actionBusy = true;
                specimenTimer.resetTimer();
                setSpecimenState(2);
            case 2:
                if(specimenTimer.getElapsedTimeSeconds() > 0) {
                    actionBusy = false;
                    setSpecimenState(-1);
                }
                break;
        }
    }

    public void setSpecimenState(int x) {
        specimenState = x;
    }

    public void startSpecimen() {
        if(actionNotBusy()) {
            setSpecimenState(1);
        }
    }
/*
    public void intake() {
        switch (intakeState) {
            case 1:
                actionBusy = true;
                claw.open();
                intakeTimer.resetTimer();
                setTransferState(2);
                break;
            case 2:
                if(intakeTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.transfer();
                    claw.transfer();
                    intake.pivotTransfer();
                    intake.spinStop();
                    speclift.toTransfer();
                    claw.open();
                    extend.toHalf();
                    intakeTimer.resetTimer();
                    setTransferState(3);
                }
                break;
            case 3:
                if (intakeTimer.getElapsedTimeSeconds() > 1) {
                    intake.pivotGround();
                    intake.spinIn();
                    intakeTimer.resetTimer();
                    setTransferState(4);
                }
                break;
            case 4:
                if (intakeTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.spinStop();
                    intakeTimer.resetTimer();
                    actionBusy = false;
                    setTransferState(-1);
                }
                break;
        }
    }

    public void setIntakeState(int x) {
        intakeState = x;
    }

    public void startIntake() {
        if (actionNotBusy()) {
            setIntakeState(1);
        }
    }

 */

    public void park() {
        switch (parkState) {
            case 1:
                actionBusy = true;
                parkTimer.resetTimer();
                setParkState(2);
                break;
            case 2:
                if(parkTimer.getElapsedTimeSeconds() > 0.5) {
                    specLift.toPark();
                    parkTimer.resetTimer();
                    actionBusy = false;
                    setTransferState(-1);
                }
                break;
        }
    }

    public void setParkState(int x) {
        parkState = x;
    }

    public void startPark() {
        if (actionNotBusy()) {
            setParkState(1);
        }
    }

    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

    public void telemetryUpdate() {
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Action Busy?: ", actionBusy);
        //telemetry.addData("speclift Pos", speclift.getPos());
        //telemetry.addData("Extend Pos", extend.leftExtend.getPosition());
        //telemetry.addData("Extend Limit", extend.extendLimit);
        //   telemetry.addData("Intake Spin State", intakeSpinState);
        //   telemetry.addData("Intake Pivot State", intakePivotState);
        telemetry.update();
    }
}