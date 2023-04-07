package frc.robot;

import java.lang.management.ClassLoadingMXBean;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** Controls driving, braking, & switching between drives */
public class DriveTrain {

    DoubleSolenoid driveSol;

    // Declaring the wheel motors
    public Motor LF1;
    public Motor LB1;
    public Motor RF1;
    public Motor RB1;
    public Motor LF2;
    public Motor LB2;
    public Motor RB2;
    public Motor RF2;

    boolean sniperMode = false;

    boolean isInTractionDrive;
    boolean tryBalancing = false;
    NavX navi;
    Ghloe ghloe;
    Limelight limelight;

    double slip = 0.5;
    double balanceTarg = 0;

    public double targetHeading = 0;
    double fieldZero;

    double lastLoopAngle = 0;
    boolean isLastAngleValid = false;
    boolean requestBrakeMode = false;
    boolean firstBrakeTime = false;

    boolean balanceFirstTime = true;
    Vector lfWheelVector = new Vector(Math.sqrt(2), Math.sqrt(2));
    Vector lbWheelVector = new Vector(-Math.sqrt(2), Math.sqrt(2));
    Vector rfWheelVector = new Vector(-Math.sqrt(2), Math.sqrt(2));
    Vector rbWheelVector = new Vector(Math.sqrt(2), Math.sqrt(2));

    public DoubleLogEntry distanceLogLeft;
    public DoubleLogEntry distanceLogRight;

    final Interpolation deadzone;

    DoubleLogEntry lf1LogEntry;
    DoubleLogEntry lf2LogEntry;
    DoubleLogEntry lb1LogEntry;
    DoubleLogEntry lb2LogEntry;
    DoubleLogEntry rf1LogEntry;
    DoubleLogEntry rf2LogEntry;
    DoubleLogEntry rb1LogEntry;
    DoubleLogEntry rb2LogEntry;

    DoubleLogEntry startDistanceLogEntry;
    DoubleLogEntry targetDistanceLogEntry;
    DoubleLogEntry errorDistanceLogEntry;
    DoubleLogEntry currentHeadingLogEntry;
    DoubleLogEntry targetHeadingLogEntry;
    DoubleLogEntry errorHeadingLogEntry;
    DoubleLogEntry influenceLogEntry;
    DoubleLogEntry speedLogEntry;
    DoubleLogEntry averageStrafeDistanceLogEntry;

    TrapezoidProfile driveDistanceProfile;
    TrapezoidProfile.Constraints driveDistanceConstraints;
    TrapezoidProfile.State driveDistanceCurrentState;
    TrapezoidProfile.State driveDistanceGoalState;
    Timer driveDistanceProfileTimer;

    Timer accelTimer;
    double previousVelocity;
    double previousTime;

    Pose2d currentPose;

    public DifferentialDriveOdometry driveOdometry;
    public DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(0.7);

    public RamseteController ramsete = new RamseteController(2.0, 0.7);
    public final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        // 0.10965, 2.2557, 0.26994);
        // 0.10965, 2.8557, 0.26994);
        0.10965, 2.2557, 0.26994);

    // static final double tractionMotRotPerMetOLD = 5.3 / 0.31; // traction motor
    // rotations per meter
    static final double tractionMotRotPerMet = 1.0 / 0.319278 / 12.0 * 60*1.1; // traction motor rotations per meter

    public DriveTrain(NavX navi, Ghloe ghloe, Limelight limelight) {
        // 8 motors neos
        // Two double solenoids are used within the drive train on our robot.
        this.navi = navi;
        this.ghloe = ghloe; // Lights
        isInTractionDrive = true; // Sets drive state to Traction by default
        // diffDrive = new DifferentialDrive(null, null)

        this.limelight = limelight;

        // Instantiating the motors and pistons
        driveSol = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.DRIVETRAIN_SOL_FWD_ID,
                Constants.DRIVETRAIN_SOL_REV_ID);

        double p = .1;
        double i = 0;
        double d = 0;
        double f = 0;

        // TODO: Wheel unit conversions to meters
        LF1 = new Motor(Constants.FRONT_LEFT_MOTOR_1_ID, "Front Left 1", p, i, d, f, tractionMotRotPerMet);
        LB1 = new Motor(Constants.BACK_LEFT_MOTOR_1_ID, "Back Left 1", p, i, d, f, tractionMotRotPerMet);
        RF1 = new Motor(Constants.FRONT_RIGHT_MOTOR_1_ID, "Front Right 1", p, i, d, f, tractionMotRotPerMet);
        RB1 = new Motor(Constants.BACK_RIGHT_MOTOR_1_ID, "Back Right 1", p, i, d, f, tractionMotRotPerMet);

        LF2 = new Motor(Constants.FRONT_LEFT_MOTOR_2_ID, "Front Left 2", p, i, d, f, tractionMotRotPerMet);
        LB2 = new Motor(Constants.BACK_LEFT_MOTOR_2_ID, "Back Left 2", p, i, d, f, tractionMotRotPerMet);
        RF2 = new Motor(Constants.FRONT_RIGHT_MOTOR_2_ID, "Front Right 2", p, i, d, f, tractionMotRotPerMet);
        RB2 = new Motor(Constants.BACK_RIGHT_MOTOR_2_ID, "Back Right 2", p, i, d, f, tractionMotRotPerMet);

        LF1.init();
        LB1.init();
        RF1.init();
        RB1.init();

        LF2.init();
        LB2.init();
        RF2.init();
        RB2.init();

        LB1.currentLimit(55);
        LF1.currentLimit(55);
        RF1.currentLimit(55);
        RB1.currentLimit(55);

        LF2.currentLimit(55);
        LB2.currentLimit(55);
        RF2.currentLimit(55);
        RB2.currentLimit(55);

        LF1.motor.setInverted(true);
        LF2.motor.setInverted(true);
        LB1.motor.setInverted(true);
        LB2.motor.setInverted(true);

        // These motors follow their adjacent motor
        LF2.motor.follow(LF1.motor);
        LB1.motor.follow(LB2.motor);
        RF2.motor.follow(RF1.motor);
        RB1.motor.follow(RB2.motor);

        accelTimer = new Timer();
        accelTimer.start();

        turnToZero(0);

        fieldZero = 0;
        deadzone = new Interpolation(new Point[] {
                new Point(-1, -1),
                new Point(-0.1, 0),
                new Point(0.1, 0),
                new Point(1, 1),
        });

        distanceLogLeft = new DoubleLogEntry(DataLogManager.getLog(), "Left Drive Distance");
        distanceLogRight = new DoubleLogEntry(DataLogManager.getLog(), "Right Drive Distance");

        try {
            lf1LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Left Front Drive Motor 1");
            lf2LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Left Front Drive Motor 2");
            lb1LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Left Back Drive Motor 1");
            lb2LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Left Back Drive Motor 2");

            rf1LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Right Front Drive Motor 1");
            rf2LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Right Front Drive Motor 2");
            rb1LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Right Back Drive Motor 1");
            rb2LogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Right Back Drive Motor 2");

            startDistanceLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Start Distance");
            targetDistanceLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Target Distance");
            errorDistanceLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Distance Error");
            currentHeadingLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Current Heading");
            targetHeadingLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Target Heading");
            errorHeadingLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Heading Error");
            influenceLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Influence");
            speedLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Speed");
            averageStrafeDistanceLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "Average Strafe Distance");
        } catch (Exception e) {
            // TODO: handle exception
        }

        currentPose = new Pose2d(
            new Translation2d(0, 0),
            navi.currentRotation());

        driveOdometry = new DifferentialDriveOdometry(
                navi.currentRotation(), LF1.getPosition(), RF1.getPosition(), new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d(0)));
        
    }

    // Restricts the value to the range to [min <= value <= max]
    public static double clamp(double value, double min, double max) {
        if (value < min) {
            value = min;
        } else if (value > max) {
            value = max;
        }
        return value;
    }

    /*
     * public void `ght(double speed, double distance, double
     * marginOfError, double compassAngle) {
     * //maskedLeftMotor.setMask(false);
     * //maskedRightMotor.setMask(false);
     * double leftDistance = LF1.getPosition();
     * double rightDistance = RF1.getPosition();
     * double distanceCovered = (leftDistance + rightDistance) / 2.0;
     * double slowdrive = -0.5;
     * // driveweee
     * if (speed < 0) {
     * slowdrive = -slowdrive;
     * }
     * }
     */

    public void arcadeDrive(double y, double x) {
        double left, right;
        left = -y;
        right = -y;

        left = left + x;
        right = right - x;

        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

        LF1.setPercentOutput(left);
        LB2.setPercentOutput(left);
        RF1.setPercentOutput(right);
        RB2.setPercentOutput(right);
    }

    public void tractionPID(double y, double r) {

        double currentHeading = navi.currentHeading();

        targetHeading = DriveTrain.clamp(targetHeading, currentHeading - 45, currentHeading + 45);

        double error = targetHeading - currentHeading;

        double output = clamp(error * (1.0 / 60.0), -0.8, 0.8) + 0.1 * r;

        arcadeDrive(y, output);
    }

    public void mechanumDrive(double x, double y, double r) {

        LF1.setPercentOutput(-y + r + x);
        LB2.setPercentOutput(-y + r - x);
        RF1.setPercentOutput(-y - r - x);
        RB2.setPercentOutput(-y - r + x);
    }

    public void mechanumPID(double x, double y, double r) {

        double currentHeading = navi.currentHeading();

        targetHeading = DriveTrain.clamp(targetHeading, currentHeading - 45, currentHeading + 45);

        double error = targetHeading - currentHeading;

        double output = clamp(error * (1.0 / 50.0), -0.8, 0.8) + 0.17 * r;

        mechanumDrive(x, y, output);
    }

    public void fieldCenteredMechanum(double x, double y, double rx, double ry) {
        // SmartDashboard.putNumber("rx", rx);
        // SmartDashboard.putNumber("ry", ry);
        // double deadzone = 0.4;
        // if ((rx < deadzone && rx > -deadzone) && (ry < deadzone && ry > -deadzone)) {
        // rx = 0;
        // ry = 0;
        // } else {
        // double angle = Math.toDegrees(Math.tanh(ry/rx));
        // if (rx == 0 && ry <= 0) {
        // targetHeading = 0;
        // } else if (rx == 0 && ry > 0) {
        // targetHeading = 180;
        // } else {
        // if (rx > 0 && ry >= 0) {
        // turnToZero(90.0 + angle);
        // } else if (rx > 0 && ry < 0) {
        // turnToZero(90.0 + angle);
        // } else if (rx < 0 && ry >= 0) {
        // turnToZero(-90.0 + angle);
        // } else if (rx < 0 && ry < 0) {
        // turnToZero(-90.0 + angle);
        // }

        // }
        // }
        // SmartDashboard.putNumber("Target Heading Tester", targetHeading);

        Translation2d projected = new Translation2d(x, -y)
                .rotateBy(new Rotation2d(Math.toRadians(navi.currentHeading())));

        mechanumPID(projected.getX(), -projected.getY(), rx);

    }

    public void curvatureDrive(double y, double x, boolean allowTurnInPlace) {
        double left, right;
        left = -y;
        right = -y;

        left = left + x;
        right = right - x;

        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

    }

    public void drive(double x, double y, double rx, double ry) {
        double pitch = navi.getPitch();

        x = deadzone.interpolate(x);
        y = deadzone.interpolate(y);
        rx = deadzone.interpolate(rx);
        ry = deadzone.interpolate(ry);

        SmartDashboard.putNumber("targetHeading", targetHeading);

        if (sniperMode) {
            x /= 2.0;
            y /= 2.0;
            rx /= 2.0;
            ry /= 2.0;
        }

        if (pitch < -26) {
            ghloe.isFallingOver = true;
            arcadeDrive(.6, 0);
        } else if (pitch > 26) {
            ghloe.isFallingOver = true;
            arcadeDrive(-.6, 0);
        } else {
            ghloe.isFallingOver = false;

            if (tryBalancing) {
                ghloe.isBalancing = true;
                if (balanceFirstTime) {
                    zeroEncoders();
                    balanceTarg = 0;
                    balanceFirstTime = false;
                }
                doTheBalanceStuff();
            } else {

                if (requestBrakeMode) {
                    ghloe.isBrakeMode = true;
                    if (firstBrakeTime) {
                        zeroEncoders();
                        firstBrakeTime = false;
                    }
                    brakeMode();
                } else {
                    ghloe.isBrakeMode = false;
                    firstBrakeTime = true;

                    balanceFirstTime = true;
                    ghloe.isBalancing = false;

                    if (limelight.lState == LimeState.GREEN || limelight.lState == LimeState.APRIL) {
                        x -= (limelight.tx * 0.02);
                    } else if (limelight.lState == LimeState.FEEDER && limelight.sideTV > 0.5) {
                        // It is still changing "x" because this is before field oriented control
                        x += (limelight.sideTX * 0.02);
                        double targetTA = 0.7;
                        double ks = 0.04;
                        double taError = (limelight.getSideTA() - targetTA);
                        y += (ks * Math.signum(taError)) + (taError * 1);

                        if (Math.abs(taError) < 0.05) {
                            ghloe.isFallingOver = true;
                        }
                    }

                    clamp(x, -1, 1);

                    if (isInTractionDrive == true) {
                        targetHeading += rx * (270.0 / 50.0);
                        tractionPID(y, rx);
                        // targetHeading = navi.currentHeading();
                    } else {
                        targetHeading += rx * (360.0 / 50.0);
                        // mechanumPID(x, y, r);
                        fieldCenteredMechanum(x, y, rx, ry);
                    }
                }

            }
        }

    }

    public void turnToZero(double offset) {
        double currentHeading = navi.currentHeading();
        int loopDirection;

        if (currentHeading > fieldZero) {
            loopDirection = 360;
        } else {
            loopDirection = -360;
        }

        double loopVariable = fieldZero + offset; // Current multiple of 360
        double lastLoopVariable = fieldZero + offset - loopDirection; // Last multiple of 360

        while (Math.abs(currentHeading) > Math.abs(loopVariable)) {
            lastLoopVariable = loopVariable;
            loopVariable += loopDirection;
        }

        if (Math.abs(currentHeading - loopVariable) < Math.abs(currentHeading - lastLoopVariable)) {
            targetHeading = loopVariable;
        } else {
            targetHeading = lastLoopVariable;
        }
    }

    public void printDriveDistance() {
        // double leftDistance = LF1.getPosition();
        // double rightDistance = RF1.getPosition();
        // SmartDashboard.putNumber("Left drive distance", leftDistance);
        // SmartDashboard.putNumber("Right drive distance", rightDistance);
        SmartDashboard.putNumber("Mechanum TargetHeading", targetHeading);
    }

    public void zeroEncoders() {
        LF1.resetEncoder();
        RF1.resetEncoder();
        LB1.resetEncoder();
        RB1.resetEncoder();

        LF2.resetEncoder();
        RF2.resetEncoder();
        LB2.resetEncoder();
        RB2.resetEncoder();
    }

    public double getAverageVelocity() {
        return (LF1.getVelocity() + RF1.getVelocity()) / 2.0;
    }

    public double getAverageDistance() {
        double leftDistance = LF1.getPosition();
        double rightDistance = RF1.getPosition();
        double distanceCovered = (leftDistance + rightDistance) / 2.0;
        return distanceCovered;
    }

    public void driveShift() { // Run Function
        if (isInTractionDrive) {
            driveSol.set(Value.kForward); // Traction Drive
        } else {
            driveSol.set(Value.kReverse); // Mechanum Drive
        }
    }

    public void run() {
        driveShift();
        robotVector();

        currentPose = driveOdometry.update(navi.currentRotation(), LF1.getPosition(), RF1.getPosition());

        // SmartDashboard.putNumber("Pose X Position", currentPose.getX());
        // SmartDashboard.putNumber("Pose Y Position", currentPose.getY());
        // SmartDashboard.putNumber("Pose Rotation", currentPose.getRotation().getDegrees());

        try {
            lf1LogEntry.append(LF1.getPosition());
            lf2LogEntry.append(LF2.getPosition());
            lb1LogEntry.append(LB1.getPosition());
            lb2LogEntry.append(LB2.getPosition());
            rf1LogEntry.append(RF1.getPosition());
            rf2LogEntry.append(RF2.getPosition());
            rb1LogEntry.append(RB1.getPosition());
            rb2LogEntry.append(RB2.getPosition());
        } catch (Exception e) {
            // TODO: handle exception
        }

    }

    /**
     * True is traction, false is Mechanum
     */
    public void setDriveState(boolean state) {
        isInTractionDrive = state; // Make a toggle in Robot.java
    }

    public void setBalancing(boolean state) {
        tryBalancing = state;
    }

    public boolean getDriveState() {
        return isInTractionDrive;
    }

    public double getLeftFrontEncoderValues() {
        return LF1.getPosition();
    }

    public double getLeftBackEncoderValues() {
        return LB2.getPosition();
    }

    public double getRightFrontEncoderValues() {
        return RF1.getPosition();
    }

    public double getRightBackEncoderValues() {
        return RB2.getPosition();
    }

    public double getLeftEncoderValues() {
        return (getLeftFrontEncoderValues() + getLeftBackEncoderValues() / 2.0);
    }

    public double getRightEncoderValues() {
        return (getRightFrontEncoderValues() + getRightBackEncoderValues() / 2.0);
    }

    public double getDriveEncoderValues() {
        return ((getRightEncoderValues() + getLeftEncoderValues()) / 2.0);
    }

    /**
     * 
     * @param startDistance  - Encoder values when the drive starts
     * @param targetDistance - Distance you want to travel (relative) Units:
     * @param targetCompass  - Heading you want to be at (absolute)
     * @param speed          - Speed you want to travel at
     * @param slowDown       - When this distance from the end is reached, the robot
     *                       will start to slow down
     * @param driveMoE       - Distance margin of error
     * @param headingMoE     - Heading margin of error
     * @param maxTurnSpeed   - Clamps max turn power/speed
     * @return True if finished, False if still moving or turning
     */
    public boolean driveDistance(double startDistance, double targetDistance, double targetCompass,
            double speed, double slowDown, double driveMoE, double headingMoE, double maxTurnSpeed) {

        double driveP = 1; // TODO: Might need to tune
        double turnP = 1.0 / 60.0; // Tuned (Go max speed until 60 degrees away from target)

        double driveError = targetDistance + startDistance - getDriveEncoderValues();
        double headError = targetCompass - navi.currentHeading();
        double influence = turnP * headError;

        influence = clamp(influence, -maxTurnSpeed, maxTurnSpeed);

        speed = DriveTrain.clamp((driveError / slowDown * driveP * speed), -Math.abs(speed), Math.abs(speed));

        SmartDashboard.putNumber("Start Distance", startDistance);
        SmartDashboard.putNumber("Target Distance", targetDistance);
        SmartDashboard.putNumber("Target Compass", targetCompass);
        SmartDashboard.putNumber("Drive Error", driveError);
        SmartDashboard.putNumber("Heading Error", headError);
        SmartDashboard.putNumber("Influence", influence);
        SmartDashboard.putNumber("Speed", speed);

        try {
            startDistanceLogEntry.append(startDistance);
            targetDistanceLogEntry.append(targetDistance);
            errorDistanceLogEntry.append(driveError);
            currentHeadingLogEntry.append(navi.currentHeading());
            targetHeadingLogEntry.append(targetHeading);
            errorHeadingLogEntry.append(headError);
            influenceLogEntry.append(influence);
            speedLogEntry.append(speed);
            averageStrafeDistanceLogEntry.append(getAverageStrafeDistance());
        } catch (Exception e) {
            // TODO: handle exception
        }

        if (Math.abs(driveError) < driveMoE && Math.abs(headError) < headingMoE) {
            arcadeDrive(0, 0);
            return true;
        } else {
            arcadeDrive(-speed, influence);
            return false;
        }
    }

    // public void smartDistanceDrive(double startDistance, double targetDistance,
    // double targetCompass,
    // double speed, double driveMoE, double headingMoE){}

    public void strafeDistance(double x, double y, double distance, double MoE, double heading) {
        targetHeading = heading;
        if (Math.abs(distance - (getAverageStrafeDistance())) > MoE) {
            mechanumPID(x, y, 0);
        } else {
            mechanumPID(0, 0, 0);
        }
    }

    public double getAverageStrafeDistance() {
        return (LF1.getPosition() + RB2.getPosition()) - (LB2.getPosition() + RF1.getPosition()) / 4.0;
    }

    public void printSensors() {
        double tempVelocity = LF1.getVelocity();
        double tempTime = accelTimer.get();

        double accel = (tempVelocity - previousVelocity) / (tempTime - previousTime);

        LF1.printSensors();
        LF2.printSensors();
        LB1.printSensors();
        LB2.printSensors();
        RF1.printSensors();
        RF2.printSensors();
        RB1.printSensors();
        RB2.printSensors();
        // SmartDashboard.putBoolean("Is Traction Drive", driveSol.get() == Value.kForward);
        // SmartDashboard.putNumber("DriveEncoder Values", getDriveEncoderValues());

        // SmartDashboard.putNumber("Drivetrain Velocity", LF1.getVelocity());
        // SmartDashboard.putNumber("Drivetrain Acceleration", accel);

        previousVelocity = tempVelocity;
        previousTime = tempTime;
    }

    public void printPIDF() {
        LF1.printPIDF();
        LF2.printPIDF();
        LB1.printPIDF();
        LB2.printPIDF();
        RF1.printPIDF();
        RF2.printPIDF();
        RB1.printPIDF();
        RB2.printPIDF();
    }

    public void getPIDF() {
        LF1.setPIDFDashboard();
        LF2.setPIDFDashboard();
        LB1.setPIDFDashboard();
        LB2.setPIDFDashboard();
        RF1.setPIDFDashboard();
        RF2.setPIDFDashboard();
        RB1.setPIDFDashboard();
        RB2.setPIDFDashboard();
    }

    public void robotVector() {
        // Angle of mechanum is roughly 45 degrees

        // Vector lfResultantVector =
        // lfWheelVector.multiplyVector(slip).multiplyVector(getLeftFrontEncoderValues());
        // Vector lbResultantVector =
        // lbWheelVector.multiplyVector(slip).multiplyVector(getLeftBackEncoderValues());
        // Vector rfResultantVector =
        // rfWheelVector.multiplyVector(slip).multiplyVector(getRightFrontEncoderValues());
        // Vector rbResultantVector =
        // rbWheelVector.multiplyVector(slip).multiplyVector(getRightBackEncoderValues());
        // Vector result =
        // lfResultantVector.addVector(lbResultantVector).addVector(rfResultantVector)
        // .addVector(rbResultantVector);

        // SmartDashboard.putString("Left Front", lfResultantVector.toString());
        // SmartDashboard.putString("Left Back", lbResultantVector.toString());
        // SmartDashboard.putString("Right Front", rfResultantVector.toString());
        // SmartDashboard.putString("Right Back", rbResultantVector.toString());
        // SmartDashboard.putString("Robot Position", result.toString());
    }

    public void doTheBalanceStuff() {
        double e = -1 * navi.getPitch(); // Error? Who names a variable e?
        if (Math.abs(e - 0) > 1) {
            double deltaE = e - lastLoopAngle; // Change in Error?
            double desiredDistancePerSecAtFifteenDegreesInMeters = 0.23; // 0.35 was the speed we had before, 0.27 for
                                                                         // teeter
            double robotCodeFrequency = 50; // Hurts :(
            double desiredDistancePerLoopAtFifteenDegreesInMeters = desiredDistancePerSecAtFifteenDegreesInMeters
                    / robotCodeFrequency;
            // we want to update target by the uhhh e by the constant every loop
            double balanceP = desiredDistancePerLoopAtFifteenDegreesInMeters / 15.0; // the fifteen is in degrees
            SmartDashboard.putNumber("BalanceDOutput", deltaE * 0.003);
            balanceTarg += e * balanceP + (deltaE * 0.003);
            SmartDashboard.putNumber("BalanceTarget", balanceTarg);
        }

        LF1.setPosition(balanceTarg);
        RF1.setPosition(balanceTarg);
        LB2.setPosition(balanceTarg);
        RB2.setPosition(balanceTarg);

        lastLoopAngle = e;

    }

    /**
     * Used for balancing
     */
    public void brakeMode() {
        LF1.setPosition(0);
        RF1.setPosition(0);
        LB2.setPosition(0);
        RB2.setPosition(0);

    }

    /**
     * Used in autonomous
     */
    public void motorBrakeMode() {
        LF1.motor.setIdleMode(IdleMode.kBrake);
        LF2.motor.setIdleMode(IdleMode.kBrake);
        LB1.motor.setIdleMode(IdleMode.kBrake);
        LB2.motor.setIdleMode(IdleMode.kBrake);
        RF1.motor.setIdleMode(IdleMode.kBrake);
        RF2.motor.setIdleMode(IdleMode.kBrake);
        RB1.motor.setIdleMode(IdleMode.kBrake);
        RB2.motor.setIdleMode(IdleMode.kBrake);
    }

    public void motorCoastMode() {
        LF1.motor.setIdleMode(IdleMode.kCoast);
        LF2.motor.setIdleMode(IdleMode.kCoast);
        LB1.motor.setIdleMode(IdleMode.kCoast);
        LB2.motor.setIdleMode(IdleMode.kCoast);
        RF1.motor.setIdleMode(IdleMode.kCoast);
        RF2.motor.setIdleMode(IdleMode.kCoast);
        RB1.motor.setIdleMode(IdleMode.kCoast);
        RB2.motor.setIdleMode(IdleMode.kCoast);
    }

    public void printDriveDistanceToLog() {
        double leftDistance = LF1.getPosition();
        double rightDistance = RF1.getPosition();
        distanceLogLeft.append(leftDistance);
        distanceLogRight.append(rightDistance);
    }

    public void setLeftPosition(double position) {
        LF1.setPosition(position);
        LB2.setPosition(position);
    }

    public void setRightPosition(double position) {
        RF1.setPosition(position);
        RB2.setPosition(position);
    }

    public void driveVoltage(double left, double right) {
        LF1.setVoltage(left);
        LB2.setVoltage(left);
        RF1.setVoltage(right);
        RB2.setVoltage(right);
    }

    public void resetOdometryPose(Pose2d pose) {
        driveOdometry.resetPosition(navi.currentRotation(), LF1.getPosition(), RF1.getPosition(), pose);
    }

}
