
package frc.robot;

import javax.swing.text.AbstractDocument.BranchElement;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class controls arm extension, pivot, and brake
 */
public class Arm {

    public static final double ARM_RATIO = 28.0/49.0;

    // Arm position constants
    public static final double LVL_2_PIVOT_CUBE = 136.8;
    public static final double LVL_3_PIVOT_CUBE = 124.9;

    public static final double LVL_2_PIVOT_CONE = 105.8;
    public static final double LVL_3_PIVOT_CONE = 105.8; //was 108.8, then was 114.8

    public static final double LVL_2_PIVOT_CONE_SCORE = 135.3;
    public static final double LVL_3_PIVOT_CONE_SCORE = 129.0; // was 124.2

    public static final double LVL_2_EXTEND_CUBE = 0;
    public static final double LVL_3_EXTEND_CUBE = 45.7;

    public static final double LVL_2_EXTEND_CONE = 26.5; // old was 23.5
    public static final double LVL_3_EXTEND_CONE = 82.0; // old was 78.4

    public static final double LVL_1_REVERSE_PIVOT_CUBE_SHORT = -221.5;
    public static final double LVL_1_PIVOT_CUBE = 190; //

    public static final double LVL_2_REVERSE_PIVOT_CUBE = -LVL_2_PIVOT_CUBE;
    public static final double LVL_3_REVERSE_PIVOT_CUBE = -LVL_3_PIVOT_CUBE;

    public static final double LVL_1_REVERSE_PIVOT_CONE_SHORT = LVL_1_REVERSE_PIVOT_CUBE_SHORT;
    public static final double LVL_1_PIVOT_CONE = LVL_1_PIVOT_CUBE;

    public static final double LVL_2_REVERSE_PIVOT_CONE = -LVL_2_PIVOT_CONE;
    public static final double LVL_3_REVERSE_PIVOT_CONE = -LVL_3_PIVOT_CONE;

    public static final double LVL_2_REVERSE_PIVOT_CONE_SCORE = -LVL_2_PIVOT_CONE_SCORE;
    public static final double LVL_3_REVERSE_PIVOT_CONE_SCORE = -LVL_3_PIVOT_CONE_SCORE;

    public static final double LVL_1_REVERSE_EXTEND_CUBE_SHORT = 0;
    public static final double LVL_1_EXTEND_CUBE = 0; // 

    public static final double LVL_2_REVERSE_EXTEND_CUBE = LVL_2_EXTEND_CUBE;
    public static final double LVL_3_REVERSE_EXTEND_CUBE = LVL_3_EXTEND_CUBE;

    public static final double LVL_1_REVERSE_EXTEND_CONE_SHORT = LVL_1_REVERSE_EXTEND_CUBE_SHORT;
    public static final double LVL_1_EXTEND_CONE = LVL_1_EXTEND_CUBE;

    public static final double LVL_2_REVERSE_EXTEND_CONE = LVL_2_EXTEND_CONE;
    public static final double LVL_3_REVERSE_EXTEND_CONE = LVL_3_EXTEND_CONE;

    public static final double SINGLE_PIVOT_FEEDER = -120.0;
    public static final double DOUBLE_PIVOT_FEEDER = 115;

    public static final double SINGLE_REVERSE_PIVOT_FEEDER = -SINGLE_PIVOT_FEEDER;
    public static final double DOUBLE_REVERSE_PIVOT_FEEDER = -DOUBLE_PIVOT_FEEDER;

    public static final double SINGLE_EXTEND_FEEDER = 0;
    public static final double DOUBLE_EXTEND_FEEDER = 26.5;

    public static final double SINGLE_REVERSE_EXTEND_FEEDER = SINGLE_EXTEND_FEEDER;
    public static final double DOUBLE_REVERSE_EXTEND_FEEDER = DOUBLE_EXTEND_FEEDER;

    public static final double LVL_1_SIDE_CONE_PICKUP_PIVOT = LVL_1_REVERSE_PIVOT_CUBE_SHORT;
    public static final double LVL_1_SIDE_CONE_PICKUP_EXTEND = 0;

    public static final double PIVOT_STOW = 0;
    public static final double EXTEND_STOW = 0;

    public static final double PIVOT_MOE = 3.5;
    public static final double EXTEND_MOE = 2;

    double pivotSetPoint = 0;
    double extension = 0;
    double adjustExtension = 0;
    double adjustPivot = 0;

    double pivotP = 0.1;
    double pivotI = 0;
    double pivotD = 0;
    double pivotF = 0;

    double extendP = 0.3;
    double extendI = 0;
    double extendD = 0;
    double extendF = 0.0;

    Solenoid pivotBrake;
    Solenoid stndoBrake;
    Motor leftPivot;
    Motor rightPivot;
    Motor retractionMotor;

    Interpolation retractionLookupTable;

    Timer timerRetraction;
    Timer timerPivot;

    DoubleLogEntry leftPivotPositionLogEntry;
    DoubleLogEntry rightPivotPositionLogEntry;
    DoubleLogEntry targetPivotLogEntry;
    BooleanLogEntry pivotBrakeLogEntry;
    DoubleLogEntry extensionPositionLogEntry;
    DoubleLogEntry extensionTargetLogEntry;
    BooleanLogEntry extensionBrakeLogEntry;

    TrapezoidProfile pivotTrapProfile;
    TrapezoidProfile.Constraints pivotTrapConstraints;
    TrapezoidProfile.State pivotTrapCurrentState;
    TrapezoidProfile.State pivotTrapGoalState;
    Timer pivotTrapTimer;

    public Arm() {
        // 750 motor rotations per arm rotation
        // 1 arm rotation = 360 degrees
        // Thus the units as degrees
        // tickPerDegree = 750.0 / 360.0;

        // Creating left and right pivot motors
        leftPivot = new Motor(Constants.LEFT_PIVOT_MOTOR_ID, "leftPivotMotor",
                pivotP, pivotI, pivotD, pivotF, ARM_RATIO);
        leftPivot.init();

        rightPivot = new Motor(Constants.RIGHT_PIVOT_MOTOR_ID, "rightPivotMotor",
                pivotP, pivotI, pivotD, pivotF, ARM_RATIO);
        rightPivot.init();

        rightPivot.motor.setInverted(true);

        // Creating retraction motor
        retractionMotor = new Motor(Constants.ARM_EXTENSION_MOTOR_ID, "retractionMotor",
                extendP, extendI, extendD, extendF, 1);
        retractionMotor.init();

        // Makes right pivot motor follow left pivot motor
        rightPivot.motor.follow(leftPivot.motor, true);


        pivotTrapConstraints = new Constraints(225, 1000);
        pivotTrapCurrentState = new State(0, 0);
        pivotTrapGoalState = new State(0, 0);
        pivotTrapProfile = new TrapezoidProfile(pivotTrapConstraints, pivotTrapGoalState, pivotTrapCurrentState);
        pivotTrapTimer = new Timer();


        // Assigns the IDs of the solenoids
        pivotBrake = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.PIVOT_BRAKE_ID);
        // stndoBrake is extension solenoid brake
        stndoBrake = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.EXTENSION_BRAKE_ID);

        Point[] points = {
                new Point(0, 0),
                new Point(100, 100),
        };
        retractionLookupTable = new Interpolation(points);

        timerRetraction = new Timer();
        timerPivot = new Timer();

        timerRetraction.start();
        timerPivot.start();

        try {
            leftPivotPositionLogEntry  = new DoubleLogEntry(DataLogManager.getLog(), "Left Pivot Position");
            rightPivotPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Right Pivot Position");
            targetPivotLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Target Pivot Position");
            pivotBrakeLogEntry = new BooleanLogEntry(DataLogManager.getLog(), "Pivot Brake");
            extensionPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Extension Position");
            extensionTargetLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Extension Target");
            extensionBrakeLogEntry = new BooleanLogEntry(DataLogManager.getLog(), "Extension Brake");
        } catch (Exception e) {
            // TODO: handle exception
        }
        
    }

    public void run() {
        // Runs constantly, sets position points for the pivot brake to check and stops
        // once met

        TrapezoidProfile.State state = pivotTrapProfile.calculate(pivotTrapTimer.get());

        leftPivot.setPosition(DriveTrain.clamp(state.position + adjustPivot, -236, 193));
        rightPivot.setPosition(DriveTrain.clamp(state.position + adjustPivot, -236, 193));

        // leftPivot.setPosition(PIVOT_STOW);
        // rightPivot.setPosition(PIVOT_STOW);

        SmartDashboard.putNumber("Trapeziodal Position", state.position);

        double BRAKE_PIVOT_MOE = PIVOT_MOE;

        if (!pivotBrake.get()) {
            BRAKE_PIVOT_MOE = PIVOT_MOE;
        } else {
            BRAKE_PIVOT_MOE = PIVOT_MOE/2.0;
        }

        if (leftPivot.isAtSetpoint(BRAKE_PIVOT_MOE)) {
            if (Math.abs(pivotSetPoint - PIVOT_STOW) < BRAKE_PIVOT_MOE) {
                pivotBrake.set(false); // Brake engaged
                leftPivot.setPIDF(0, 0, 0, 0);
                rightPivot.setPIDF(0, 0, 0, 0);
                timerPivot.reset();
            }
        } else {
            pivotBrake.set(true); // Brake release
            if (timerPivot.get() > 0.075) {
                leftPivot.setPIDF(pivotP, pivotI, pivotD, pivotF);
                rightPivot.setPIDF(pivotP, pivotI, pivotD, pivotF);
            }
            
            // Log
            try {
                leftPivotPositionLogEntry.append(leftPivot.getPosition());
                rightPivotPositionLogEntry.append(rightPivot.getPosition());
                targetPivotLogEntry.append(pivotSetPoint);
                pivotBrakeLogEntry.append(pivotBrake.get());
                extensionPositionLogEntry.append(retractionMotor.getPosition());
                extensionTargetLogEntry.append(extension);
                extensionBrakeLogEntry.append(stndoBrake.get());
            } catch (Exception e) {
                // TODO: handle exception
            }
            

        }

        double BRAKE_EXTENSION_MOE = EXTEND_MOE;

        if (!stndoBrake.get()) {
            BRAKE_EXTENSION_MOE = EXTEND_MOE;
        } else {
            BRAKE_EXTENSION_MOE = EXTEND_MOE/2.0;
        }
        retractionMotor.setPosition(extension + adjustExtension);
        if (retractionMotor.isAtSetpoint(BRAKE_EXTENSION_MOE)) {
            if (Math.abs(extension - EXTEND_STOW) < BRAKE_EXTENSION_MOE) {
                stndoBrake.set(false); // Brake engaged
                retractionMotor.setPIDF(0, 0, 0, 0);
                timerRetraction.reset();
            }
        } else {
            stndoBrake.set(true); // Brake released
            if (timerRetraction.get() > 0.075) {
                retractionMotor.setPIDF(extendP, extendI, extendD, extendF);
            }
        }

        SmartDashboard.putNumber("Pivot Target", pivotSetPoint);
        SmartDashboard.putNumber("Retraction Target", extension);
    }

    public void zeroEncoders() {
        leftPivot.resetEncoder();
        rightPivot.resetEncoder();
        retractionMotor.resetEncoder();
    }

    public void setPivot(double angle, double adjustPiv) {
        // sets the pivot
        angle = DriveTrain.clamp(angle, -236, 193); // Arm safety
        SmartDashboard.putNumber("armPositionReal", leftPivot.getPosition());
        SmartDashboard.putNumber("armVelocityReal", leftPivot.getVelocity());
        if (pivotSetPoint != angle) {
            pivotTrapGoalState = new State(angle, 0);
            pivotTrapCurrentState = new State(leftPivot.getPosition(), leftPivot.getVelocity());
            pivotTrapProfile = new TrapezoidProfile(pivotTrapConstraints, pivotTrapGoalState, pivotTrapCurrentState);
            pivotTrapTimer.reset();
            pivotTrapTimer.start();
        }
        pivotSetPoint = angle;
        adjustPivot = adjustPiv;
    }

    public void setExtension(double extendo, double adjustExt) {
        // sets the extension
        DriveTrain.clamp(extendo, -2, 90);
        extension = extendo;
        adjustExtension = adjustExt;
    }

    public double armPivotPosition() {
        // Returns average of left and right pivot position
        double avg = (leftPivot.getPosition() + rightPivot.getPosition()) / 2.0;
        return avg;
    }

    public void printSensors() {
        leftPivot.printSensors();
        rightPivot.printSensors();
        SmartDashboard.putBoolean("Pivot Brake State", pivotBrake.get());
        SmartDashboard.putBoolean("Extension Brake State", stndoBrake.get());
        SmartDashboard.putNumber(retractionMotor.name + " Position", getRetraction());

    }

    public void printPIDF() {
        leftPivot.printPIDF();
        rightPivot.printPIDF();
        retractionMotor.printPIDF();
    }

    public void getPIDF() {
        leftPivot.setPIDFDashboard();
        rightPivot.setPIDFDashboard();
        retractionMotor.setPIDFDashboard();

        pivotP = leftPivot.P;
        pivotI = leftPivot.I;
        pivotD = leftPivot.D;
        pivotF = leftPivot.F;

        extendP = retractionMotor.P;
        extendI = retractionMotor.I;
        extendD = retractionMotor.D;
        extendF = retractionMotor.F;
    }

    public boolean checkPosition(double normalMoE, double retractMoE) {
        return (Math.abs(pivotSetPoint - leftPivot.getPosition()) < normalMoE) && retractionMotor.isAtSetpoint(retractMoE);
    }

    public double getRetraction() {
        return retractionLookupTable.interpolate(retractionMotor.getPosition());
    }
}
