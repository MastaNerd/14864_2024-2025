package config.subsystem;

import static config.util.RobotConstants.*;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SpecArmSubsystem {

    public enum ArmState {
        SCORING, INIT, SPECGRAB, SPECPLACE, SPECBACKPLACE, FOLDED, MIDDLE, TRANSFER, SPECPICKUP
    }

    public Servo LeftArmServo, RightArmServo, LeftDiffyServo, RightDiffyServo, ClawServo;

    public ArmState state;
    public ElapsedTime timer;

    public SpecArmSubsystem(HardwareMap hardwareMap, ArmState state) {
        LeftArmServo = hardwareMap.get(Servo.class, "Left Arm Servo");
        RightArmServo = hardwareMap.get(Servo.class, "Right Arm Servo");
        LeftDiffyServo = hardwareMap.get(Servo.class, "Left Diffy");
        RightDiffyServo = hardwareMap.get(Servo.class, "Right Diffy");
        ClawServo = hardwareMap.get(Servo.class, "Claw Servo");

        LeftDiffyServo.setDirection(Servo.Direction.REVERSE);
        LeftArmServo.setDirection(Servo.Direction.REVERSE);
        this.state = state;
    }

    // State //
    public void setState(ArmState armState) {
        if (armState == ArmState.INIT) {
            ClawServo.setPosition(armClawOpen);
            this.state = ArmState.INIT;
        }else if (armState == ArmState.MIDDLE) {
            LeftArmServo.setPosition(armMiddle);
            RightArmServo.setPosition(armMiddle);
        }else if (armState == ArmState.SCORING) {
            LeftArmServo.setPosition(armDeposit);
            RightArmServo.setPosition(armDeposit);
            this.state = ArmState.SCORING;
        }else if (armState == ArmState.SPECGRAB) {
            LeftArmServo.setPosition(armSpecimenGrab);
            RightArmServo.setPosition(armSpecimenGrab);
            this.state = ArmState.SPECGRAB;
        }else if (armState == ArmState.SPECPLACE) {
            LeftArmServo.setPosition(armSpecimenPlace);
            RightArmServo.setPosition(armSpecimenPlace);
            this.state = ArmState.SPECPLACE;
        }else if (armState == ArmState.SPECBACKPLACE) {
            LeftArmServo.setPosition(armSpecimenBackPlace);
            RightArmServo.setPosition(armSpecimenBackPlace);
            this.state = ArmState.SPECBACKPLACE;
        }else if (armState == ArmState.SPECPICKUP) {
            LeftArmServo.setPosition(armSpecimenPickup);
            RightArmServo.setPosition(armSpecimenPickup);
            LeftDiffyServo.setPosition(lDiffySpecimenPickup);
            RightDiffyServo.setPosition(rDiffySpecimenPickup);
            this.state = ArmState.SPECPICKUP;
        }else if (armState == ArmState.FOLDED) {
            LeftArmServo.setPosition(armFolded);
            RightArmServo.setPosition(armFolded);
            LeftDiffyServo.setPosition(lDiffyFolded);
            RightDiffyServo.setPosition(rDiffyFolded);
            this.state = ArmState.FOLDED;
        }else if (armState == ArmState.TRANSFER) {
            LeftArmServo.setPosition(armTransfer);
            RightArmServo.setPosition(armTransfer);
            this.state = ArmState.TRANSFER;
        }
    }

    public void switchState() {
        if (state == ArmState.TRANSFER) {
            setState(ArmState.SCORING);
        } else if (state == ArmState.SCORING) {
            setState(ArmState.TRANSFER);
        }
    }

    // Preset //

    public void transfer() {
        setState(ArmState.TRANSFER);
    }

    public void score() {
        setState(ArmState.SCORING);
    }

    public void specPickup() {
        setState(ArmState.SPECPICKUP);
    }
    public void folded() {
        setState(ArmState.FOLDED);
    }
    public void pickup() {
        setState(ArmState.SPECPICKUP);
    }

    public void specGrab() {
        setState(ArmState.SPECGRAB);
    }

    public void specPlace() {
        setState(ArmState.SPECPLACE);
    }

    public double getCurrPos() {
        return LeftArmServo.getPosition();
    }

    public void toMiddle() {
        setState(ArmState.MIDDLE);
    }

    public void initArm() {setState(ArmState.INIT);}

    // Util //
    public void openClaw() {
        ClawServo.setPosition(armClawOpen);
    }

    public void closeClaw() {
        ClawServo.setPosition(armClawClose);
    }

    public void setPos(double armPos, double lDiffyPos, double rDiffyPos) {
        LeftArmServo.setPosition(armPos);
        RightArmServo.setPosition(armPos);
        LeftDiffyServo.setPosition(lDiffyPos);
        RightDiffyServo.setPosition(rDiffyPos);
    }

    public void pickupBlock(Timer clawTimer) {
        LeftArmServo.setPosition(0.79);
        RightArmServo.setPosition(0.79);
        if(clawTimer.getElapsedTimeSeconds() > 0.1){
            ClawServo.setPosition(armClawClose);
        }
        if(clawTimer.getElapsedTimeSeconds() > 0.5){
            LeftArmServo.setPosition(0.79);
            RightArmServo.setPosition(0.79);
        }
    }

    // Init + Start //
    public void init() {
        initArm();
        timer = new ElapsedTime();
    }

    public void start() {
        transfer();
    }

}