package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.WheelStates;

/** Controls intake wheels and the wrist rotation of the claw */
public class Claw {
    Motor wrist;
    Motor weels;

    Ghloe ghloe;

    boolean isRightSideUp;
    boolean wristNinty;

    double backArmDeadband;
    double frontArmDeadband;

    double weelPower = 0.0;

    public WheelStates wheelState = WheelStates.slowIn;

    // TODO: Assign values
    public static final double CLAW_FACE_UP = 0;
    public static final double CLAW_FACE_DOWN = 0;

    DoubleLogEntry rotationSpeedLogEntry;
    DoubleLogEntry clawAnglePositionLogEntry;

    public Claw(Ghloe ghloe) {
        // Assigns wrist motor and claw wheel IDs and then creates PIDF values
        wrist = new Motor(Constants.WRIST_MOTOR_ID, "wrist", 0.2, 0, 0, 0, 100.0 / 360.0, 20); // 100 motor rotations
                                                                                               // every 360 degreece
        weels = new Motor(Constants.CLAW_WHEELS_MOTOR_ID, "weels", 0, 0, 0, 0, 1.0, 20);

        this.ghloe = ghloe;

        wrist.init();
        weels.init();
        isRightSideUp = true; // Sets default wrist position regardless of arm orientation

        backArmDeadband = -10;
        frontArmDeadband = 10;

        wrist.motor.getEncoder().setPosition(wrist.unitsToRotations(90));

        weels.motor.setIdleMode(IdleMode.kBrake);

        try {
            rotationSpeedLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Wheel Speed");
            clawAnglePositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Claw Angle");
        } catch (Exception e) {
            // TODO: handle exception
        }

    }

    public void zeroEncoders() {
        wrist.resetEncoder();
        weels.resetEncoder();
    }

    public void runIntake(double power) {
        weelPower = power;
        weels.setPercentOutput(power);
    }

    public void setWristRightSideUp(boolean isRightSideUp) {
        this.isRightSideUp = isRightSideUp;
    }

    public void setWristNinty(boolean wristNinty) {
        this.wristNinty = wristNinty;
    }

    public void run(double armPosition) {
        double targetWristPosition;


        try {
            rotationSpeedLogEntry.append(weelPower);
            clawAnglePositionLogEntry.append(wrist.getPosition());
        } catch (Exception e) {
            // TODO: handle exception
        }
        

        // If the arm is behind us
        if (armPosition < backArmDeadband) {
            // And we want the wrist to be the right way up

            if (wristNinty) {
                targetWristPosition = 90;
            } else if (isRightSideUp) {
                // Then flip it over
                targetWristPosition = 180;
            } else {
                targetWristPosition = 0;
            }
            // If the arm is in front of us, same format
        } else if (armPosition > frontArmDeadband) {
            if (wristNinty) {
                targetWristPosition = 90;
            } else if (isRightSideUp) {
                targetWristPosition = 0;
            } else {
                targetWristPosition = 180;
            }
        } else {
            targetWristPosition = 90;
        }

        if (checkWeelsCurrent() && weelPower > .3) {
            ghloe.hasPiece = true;
        } else {
            ghloe.hasPiece = false;
        }
        
        wrist.setPosition(targetWristPosition);

        // Wheel Movement

        if (wheelState == WheelStates.in) {
            runIntake(.6);
            weels.currentLimit(40);
        } else if (wheelState == WheelStates.out) {
            runIntake(-.3);
            weels.currentLimit(20);
        } else if (wheelState == WheelStates.off) {
            runIntake(0);
        } else if (wheelState == WheelStates.slowIn) {
            runIntake(0.15);
            weels.currentLimit(19);
        } else if (wheelState == WheelStates.mediumIn) {
            runIntake(.3);
            weels.currentLimit(25);
        }

    }

    public boolean checkWeelsCurrent() {
        return weels.motor.getOutputCurrent() > 15;
    }

    public void printSensors() {
        wrist.printSensors();
        weels.printSensors();

    }

    public void printPIDF() {
        wrist.printPIDF();
        weels.printPIDF();
    }

    public void getPIDF() {
        wrist.setPIDFDashboard();
        weels.setPIDFDashboard();
    }

}
