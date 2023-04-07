package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Motor {
    double defaultP;
    double defaultI;
    double defaultD;
    double defaultF;

    double P;
    double I;
    double D;
    double F;
    int current;

    String name;

    double rotationsPerUnit;
    double positionTarget;

    CANSparkMax motor;
    SparkMaxPIDController pid;

    public Motor(int canID, String name, double P, double I, double D, double F, double rotationsPerUnit, int current){
        motor = new CANSparkMax(canID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        pid = motor.getPIDController();
        
        this.rotationsPerUnit = rotationsPerUnit;
        positionTarget = 0.0;

        this.current = current;
        this.name = name;
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;

        defaultP = P;
        defaultI = I;
        defaultD = D;
        defaultF = F;
    }

    public Motor(int canID, String name, double P, double I, double D, double F, double rotationsPerUnit){
        motor = new CANSparkMax(canID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        pid = motor.getPIDController();
        
        this.rotationsPerUnit = rotationsPerUnit;
        positionTarget = 0.0;

        this.current = 999; //unlimited?
        this.name = name;
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;

        defaultP = P;
        defaultI = I;
        defaultD = D;
        defaultF = F;
    }

    /**
     * Must be called after constructing the object. 
     * Sets PID and current limiting in motor controller
     */
    public void init(){
        setPIDF(defaultP, defaultI, defaultD, defaultF);
        if (current != 999) {
            currentLimit(current);
        }
    }

    /*
    * Sets PIDF to their defaults set when the motor object was created
    */
    public void resetPIDF(){
        P = defaultP;
        I = defaultI;
        D = defaultD;
        F = defaultF;
        setPIDF(P, I, D, F);
    }

    /**
     * Sets PIDF values from method input
     */
    public void setPIDF(double p, double i, double d, double f){
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);
        pid.setFF(f);
    }

    public void setPosition(double position) {
        setPosition(position, CANSparkMax.ControlType.kPosition);
    }

    public void setPosition(double position, CANSparkMax.ControlType cType) {
        positionTarget = position;
        position = unitsToRotations(position);
        pid.setReference(position, cType);
    }

    public double getPosition() {
        return rotationsToUnits(motor.getEncoder().getPosition());
    }

    public boolean isAtSetpoint(double marginOfError) {
        // If we're within the margin of error, return true, else return false
        return (Math.abs(positionTarget - getPosition()) < marginOfError);
    }

    public void setPercentOutput(double power) {
        motor.set(power);
    }

    public void setVoltage(double v) {
        motor.setVoltage(v);
    }

    public double rotationsToUnits(double rotations) {
        return rotations / rotationsPerUnit;
    }

    public double unitsToRotations(double units) {
        return units * rotationsPerUnit;
    }

    public void printPIDF() {
        SmartDashboard.putNumber(name + " P", P);
        SmartDashboard.putNumber(name + " I", I);
        SmartDashboard.putNumber(name + " D", D);
        SmartDashboard.putNumber(name + " F", F);
    }

    public void printSensors() {
        // SmartDashboard.putNumber(name + " Position", getPosition());
        // SmartDashboard.putNumber(name + " Current", motor.getOutputCurrent());

    }

    /**
     * Gets the PIDF values from the dashboard
     */
    public void setPIDFDashboard() {
        P = SmartDashboard.getNumber(name + " P", P);
        I = SmartDashboard.getNumber(name + " I", I);
        D = SmartDashboard.getNumber(name + " D", D);
        F = SmartDashboard.getNumber(name + " F", F);  
        //current =  (int)SmartDashboard.getNumber(name + " Current", current);
        setPIDF(P, I, D, F);
        //currentLimit(current);  
    }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }

    public double getVelocity() {
        return rotationsToUnits(motor.getEncoder().getVelocity()) / 60.0;
    }

    public void currentLimit(int limit){
        motor.setSmartCurrentLimit(limit);
    }




}
