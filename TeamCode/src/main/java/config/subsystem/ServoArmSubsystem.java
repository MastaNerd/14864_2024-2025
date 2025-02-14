package config.subsystem;

import static config.util.RobotConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoArmSubsystem {

    public enum ArmState {
        TRANSFER, SCORING, INIT, SPECIMENGRAB, SPECIMENPLACE, SPECIMENBACKPLACE, RETURN, TOBASKET, TOP, MIDDLE
    }

    public Servo LeftArmServo, RightArmServo, ClawWrist;
    public CRServo ClawSpinner;
    public ArmState state;
    public ElapsedTime timer;

    public ServoArmSubsystem(HardwareMap hardwareMap, ArmState state) {
        LeftArmServo = hardwareMap.get(Servo.class, "Left Arm Servo");
        RightArmServo = hardwareMap.get(Servo.class, "RightArmServo");
        ClawSpinner = hardwareMap.get(CRServo.class, "ClawSpinner");
        ClawWrist = hardwareMap.get(Servo.class, "ClawWrist");
        LeftArmServo.setDirection(Servo.Direction.REVERSE);
        ClawSpinner.setDirection(CRServo.Direction.REVERSE);
        this.state = state;

    }

    // State //
    public void setState(ArmState armState) {
        if (armState == ArmState.TRANSFER) {
            LeftArmServo.setPosition(lArmTransfer);
            RightArmServo.setPosition(rArmTransfer);
            ClawWrist.setPosition(clawWristInit);
            this.state = ArmState.TRANSFER;
        } else if (armState == ArmState.TOBASKET) {
            LeftArmServo.setPosition(lArmDeposit);
            RightArmServo.setPosition(rArmDeposit);
            ClawWrist.setPosition(clawWristInit);
        }else if (armState == ArmState.TOP) {
            LeftArmServo.setPosition(lArmDeposit);
            RightArmServo.setPosition(rArmDeposit);
            ClawWrist.setPosition(clawWristInit);
        }else if (armState == ArmState.MIDDLE) {
            LeftArmServo.setPosition(lArmMiddle);
            RightArmServo.setPosition(rArmMiddle);
            ClawWrist.setPosition(clawWristInit);
        }else if (armState == ArmState.SCORING) {
            LeftArmServo.setPosition(lArmDeposit);
            RightArmServo.setPosition(rArmDeposit);
            ClawWrist.setPosition(clawWristDeposit);
            this.state = ArmState.SCORING;
        } else if (armState == ArmState.INIT) {
            LeftArmServo.setPosition(lArmInit);
            RightArmServo.setPosition(rArmInit);
            ClawWrist.setPosition(clawWristInit);
            this.state = ArmState.INIT;
        } else if (armState == ArmState.SPECIMENGRAB) {
            LeftArmServo.setPosition(lArmSpecimenGrab);
            RightArmServo.setPosition(rArmSpecimenGrab);
            ClawWrist.setPosition(clawWristGrab);
            this.state = ArmState.SPECIMENGRAB;
        } else if (armState == ArmState.SPECIMENPLACE) {
            LeftArmServo.setPosition(lArmSpecimenPlace);
            RightArmServo.setPosition(rArmSpecimenPlace);
            ClawWrist.setPosition(clawWristPlace);
            this.state = ArmState.SPECIMENPLACE;
        }else if (armState == ArmState.RETURN) {
            LeftArmServo.setPosition(lArmSpecimenReturn);
            RightArmServo.setPosition(rArmSpecimenReturn);
            ClawWrist.setPosition(clawWristReturn);
            this.state = ArmState.RETURN;
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

    public void specimenGrab() {
        setState(ArmState.SPECIMENGRAB);
    }

    public void specimenPlace() {
        setState(ArmState.SPECIMENPLACE);
    }

    public void toReturn() {
        setState(ArmState.RETURN);
    }

    public void toMiddle() {
        setState(ArmState.MIDDLE);
    }

    public void toTop() {
        setState(ArmState.TOP);
    }

    public void toBasket() {
        setState(ArmState.TOBASKET);
    }

    public void initArm() {setState(ArmState.INIT);}

    // Util //
    public void setPos(double armPos) {
        LeftArmServo.setPosition(armPos);
        RightArmServo.setPosition(armPos);
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