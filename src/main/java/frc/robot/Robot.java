package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Autonomous;

public class Robot extends TimedRobot {
  // Defining our objects for our various classes
  XboxController driverJoystick;
  XboxController coDriverJoystick;
  public Wedge wedge;
  public Claw claw;
  public DriveTrain drive;
  public LimeState limestate = LimeState.OFF;
  public Arm arm;
  public Ghloe ghloe; // Lights
  public Limelight limelight;
  public NavX navi;
  public Autonomous auto;
  public PowerDistribution pdp;

  PneumaticHub pneu;

  @Override
  public void robotInit() {
    // this.enableLiveWindowInTest(false);
    driverJoystick = new XboxController(0);
    coDriverJoystick = new XboxController(1);

    pneu = new PneumaticHub(2);
    navi = new NavX();
    wedge = new Wedge();
    arm = new Arm();
    ghloe = new Ghloe(0, pneu, navi, this);
    limelight = new Limelight();
    drive = new DriveTrain(navi, ghloe, limelight);
    pdp = new PowerDistribution(1, ModuleType.kRev);
    claw = new Claw(ghloe);
    auto = new Autonomous(this);

    pneu.enableCompressorAnalog(100, 115); // Pressure

    // arm.printPIDF();
    // claw.printPIDF();
    // drive.printPIDF();

    drive.zeroEncoders();
    arm.zeroEncoders();

    try {
      DriverStation.startDataLog(DataLogManager.getLog());
      DataLogManager.logNetworkTables(true);
    } catch (Exception e) {
      // TODO: handle exception
    }
    new Thread(ghloe).start();
    new Thread(navi).start();

  }

  @Override
  public void robotPeriodic() {
    limelight.run();
    // navi.run(); // TODO: Rio Log Message

    arm.printSensors();
    claw.printSensors();
    drive.printSensors();
    wedge.printStatus();

    // 1 for bar, 1 for value
    SmartDashboard.putNumber("Pressure", pneu.getPressure(0));
    SmartDashboard.putNumber(" ", pneu.getPressure(0));
  }

  @Override
  public void autonomousInit() {
    auto.Init();
    drive.motorBrakeMode();
    navi.setStartingHeading();
  }

  @Override
  public void autonomousPeriodic() {
    auto.Periodic();
  }

  @Override
  public void autonomousExit() {
    drive.motorBrakeMode();
  }

  @Override
  public void teleopInit() {
    drive.motorCoastMode();
  }

  @Override
  public void teleopPeriodic() {
    double pivotTargetPosition;
    double extendTargetPosition;

    // Wedge
    if (driverJoystick.getAButton()) {
      wedge.deploy(true);
    }
    if (driverJoystick.getBButton()) {
      wedge.deploy(false);
    }

    if (driverJoystick.getXButton()) {
      drive.turnToZero(0);
    }

    if (driverJoystick.getYButton()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        drive.turnToZero(90);
      } else {
        drive.turnToZero(-90);
      }
    }

    if (driverJoystick.getStartButton()) {
      drive.setBalancing(true);
    } else {
      drive.setBalancing(false);
    }

    // Switches drive mode
    if (driverJoystick.getLeftBumper()) {
      drive.setDriveState(true); // True is traction mode
      wedge.deploy(false);
    } else if (driverJoystick.getRightBumper()) {
      drive.setDriveState(false); // False is mecanum mode
    }

    // Driver fail safe to cancel out of mechanum turn
    // Sets target heading to current heading
    if (driverJoystick.getPOV() == 0) {
      drive.targetHeading = navi.currentHeading();
    }

    // Actually driving
    double driverLY = driverJoystick.getLeftY();
    double driverLX = driverJoystick.getLeftX();
    double driverRX = driverJoystick.getRightX();
    double driverRY = driverJoystick.getRightY();
    if (driverJoystick.getRightStickButton()) {
      drive.sniperMode = true;
    } else {
      drive.sniperMode = false;
    }

    if (driverJoystick.getBackButton()) {
      drive.requestBrakeMode = true;
    } else {
      drive.requestBrakeMode = false;
    }

    drive.drive(driverLX, driverLY, driverRX, driverRY);

    // Arm Pivot & Extension
    if (coDriverJoystick.getLeftTriggerAxis() > 0.8) { // Is Reverse
      if (coDriverJoystick.getBackButton()) { // Is Cube
        if (coDriverJoystick.getAButton()) { // LVL 1 Short
          pivotTargetPosition = Arm.LVL_1_SIDE_CONE_PICKUP_PIVOT;
          extendTargetPosition = Arm.LVL_1_SIDE_CONE_PICKUP_EXTEND;
        } else if (coDriverJoystick.getXButton()) { // LVL 2
          pivotTargetPosition = Arm.LVL_2_REVERSE_PIVOT_CUBE;
          extendTargetPosition = Arm.LVL_2_REVERSE_EXTEND_CUBE;
        } else if (coDriverJoystick.getYButton()) { // LVL 3
          pivotTargetPosition = Arm.LVL_3_REVERSE_PIVOT_CUBE;
          extendTargetPosition = Arm.LVL_3_REVERSE_EXTEND_CUBE;
        } else if (coDriverJoystick.getBButton()) { // Single Feeder
          pivotTargetPosition = Arm.SINGLE_REVERSE_PIVOT_FEEDER;
          extendTargetPosition = Arm.SINGLE_REVERSE_EXTEND_FEEDER;
        } else if (coDriverJoystick.getRightStickButton()) { // Double Feeder
          pivotTargetPosition = Arm.DOUBLE_REVERSE_PIVOT_FEEDER;
          extendTargetPosition = Arm.DOUBLE_REVERSE_EXTEND_FEEDER;
        } else {
          pivotTargetPosition = Arm.PIVOT_STOW;
          extendTargetPosition = Arm.EXTEND_STOW;
        }
      } else { // Is Cone
        if (coDriverJoystick.getAButton()) { // LVL 1
          pivotTargetPosition = Arm.LVL_1_PIVOT_CONE;
          extendTargetPosition = Arm.LVL_1_EXTEND_CONE;
        } else if (coDriverJoystick.getXButton()) { // LVL 2
          if (coDriverJoystick.getRightTriggerAxis() > 0.8) {
            pivotTargetPosition = Arm.LVL_2_REVERSE_PIVOT_CONE_SCORE;
          } else {
            pivotTargetPosition = Arm.LVL_2_REVERSE_PIVOT_CONE;
          }
          extendTargetPosition = Arm.LVL_2_REVERSE_EXTEND_CONE;
        } else if (coDriverJoystick.getYButton()) { // LVL 3
          if (coDriverJoystick.getRightTriggerAxis() > 0.8) {
            pivotTargetPosition = Arm.LVL_3_REVERSE_PIVOT_CONE_SCORE;
          } else {
            pivotTargetPosition = Arm.LVL_3_REVERSE_PIVOT_CONE;
          }
          extendTargetPosition = Arm.LVL_3_REVERSE_EXTEND_CONE;
        } else if (coDriverJoystick.getBButton()) { // Single Feeder
          pivotTargetPosition = Arm.SINGLE_REVERSE_PIVOT_FEEDER;
          extendTargetPosition = Arm.SINGLE_REVERSE_EXTEND_FEEDER;
        } else if (coDriverJoystick.getRightStickButton()) { // Double Feeder
          pivotTargetPosition = Arm.DOUBLE_REVERSE_PIVOT_FEEDER;
          extendTargetPosition = Arm.DOUBLE_REVERSE_EXTEND_FEEDER;
        } else {
          pivotTargetPosition = Arm.PIVOT_STOW;
          extendTargetPosition = Arm.EXTEND_STOW;
        }
      }
    } else { // Forwards
      if (coDriverJoystick.getBackButton()) { // Is Cube
        if (coDriverJoystick.getAButton()) { // LVL 1
          pivotTargetPosition = Arm.LVL_1_SIDE_CONE_PICKUP_PIVOT;
          extendTargetPosition = Arm.LVL_1_SIDE_CONE_PICKUP_EXTEND;
        } else if (coDriverJoystick.getXButton()) { // LVL 2
          pivotTargetPosition = Arm.LVL_2_PIVOT_CUBE;
          extendTargetPosition = Arm.LVL_2_EXTEND_CUBE;
        } else if (coDriverJoystick.getYButton()) { // LVL 3
          pivotTargetPosition = Arm.LVL_3_PIVOT_CUBE;
          extendTargetPosition = (Arm.LVL_3_EXTEND_CUBE);
        } else if (coDriverJoystick.getBButton()) { // Single Feeder
          pivotTargetPosition = Arm.SINGLE_PIVOT_FEEDER;
          extendTargetPosition = (Arm.SINGLE_EXTEND_FEEDER);
        } else if (coDriverJoystick.getRightStickButton()) { // Double Feeder
          pivotTargetPosition = Arm.DOUBLE_PIVOT_FEEDER;
          extendTargetPosition = (Arm.DOUBLE_EXTEND_FEEDER);
        } else {
          pivotTargetPosition = Arm.PIVOT_STOW;
          extendTargetPosition = Arm.EXTEND_STOW;
        }
      } else { // Is Cone
        if (coDriverJoystick.getAButton()) { // LVL 1
          pivotTargetPosition = Arm.LVL_1_REVERSE_PIVOT_CONE_SHORT;
          extendTargetPosition = Arm.LVL_1_REVERSE_EXTEND_CONE_SHORT;
        } else if (coDriverJoystick.getXButton()) { // LVL 2
          if (coDriverJoystick.getRightTriggerAxis() > 0.8) {
            pivotTargetPosition = Arm.LVL_2_PIVOT_CONE_SCORE;
          } else {
            pivotTargetPosition = Arm.LVL_2_PIVOT_CONE;
          }
          extendTargetPosition = Arm.LVL_2_EXTEND_CONE;
        } else if (coDriverJoystick.getYButton()) { // LVL 3
          if (coDriverJoystick.getRightTriggerAxis() > 0.8) {
            pivotTargetPosition = Arm.LVL_3_PIVOT_CONE_SCORE;
          } else {
            pivotTargetPosition = Arm.LVL_3_PIVOT_CONE;
          }
          extendTargetPosition = Arm.LVL_3_EXTEND_CONE;
        } else if (coDriverJoystick.getBButton()) { // Single Feeder
          pivotTargetPosition = Arm.SINGLE_PIVOT_FEEDER;
          extendTargetPosition = Arm.SINGLE_EXTEND_FEEDER;
        } else if (coDriverJoystick.getRightStickButton()) { // Double Feeder
          pivotTargetPosition = Arm.DOUBLE_PIVOT_FEEDER;
          extendTargetPosition = Arm.DOUBLE_EXTEND_FEEDER;
        } else {
          pivotTargetPosition = Arm.PIVOT_STOW;
          extendTargetPosition = Arm.EXTEND_STOW;
        }
      }
    }
    if (coDriverJoystick.getAButton() && coDriverJoystick.getBackButton()) {
      claw.setWristNinty(true);
    } else {
      claw.setWristNinty(false);
    }

    // Arm adjustments once in position
    double calistaExtend = (EthanMath.map(-coDriverJoystick.getLeftY(), -1, 1, -20, 20));
    double calistaPivot = (EthanMath.map(coDriverJoystick.getLeftX(), -1, 1, -20, 20));

    // setting arm adjustments
    arm.setPivot(pivotTargetPosition, calistaPivot);
    arm.setExtension(extendTargetPosition, calistaExtend);

    // Changing LimeLight state
    if (driverJoystick.getLeftTriggerAxis() > 0.8 && driverJoystick.getRightTriggerAxis() > 0.8) {
      limestate = LimeState.FEEDER;
    }
    else if (driverJoystick.getLeftTriggerAxis() > 0.8) {
      limestate = LimeState.GREEN;
    } else if (driverJoystick.getRightTriggerAxis() > 0.8) {
      limestate = LimeState.APRIL;
    } else {
      limestate = LimeState.OFF;
    }

    limelight.setPipeline(limestate); // Sets the pipeline according to limestate

    // Intake deploy and discharge
    if (coDriverJoystick.getRightBumper()) {
      claw.wheelState = WheelStates.in;
    } else if (coDriverJoystick.getLeftBumper()) {
      claw.wheelState = WheelStates.out;
    } else {
      claw.wheelState = WheelStates.slowIn;
    }

    if (driverJoystick.getStartButtonPressed() && driverJoystick.getBackButtonPressed()) {
      navi.setStartingHeading();
    }

    // "Ghloe" Implementation (light strip)
    if (coDriverJoystick.getPOV() == 90) {
      ghloe.isRequestingCone = false;
      ghloe.isRequestingCube = true;
    } else if (coDriverJoystick.getPOV() == 270) {
      ghloe.isRequestingCone = true;
      ghloe.isRequestingCube = false;
    } else {
      ghloe.isRequestingCone = false;
      ghloe.isRequestingCube = false;
    }

    // Claw set right side up
    if (coDriverJoystick.getPOV() == 180) { // 180 means DPad down
      claw.setWristRightSideUp(false);

    } else if (coDriverJoystick.getPOV() == 0) { // 0 means DPad up
      claw.setWristRightSideUp(true);
    }

    // Get PIDF from smartdashboard
    // if (driverJoystick.getStartButton() && driverJoystick.getBackButton()
    //     || coDriverJoystick.getStartButton() && coDriverJoystick.getBackButton()) {
    //   arm.getPIDF();
    //   claw.getPIDF();
    //   drive.getPIDF();
    // }

    // Calling the run methods of all our functional mechanisms
    wedge.run(coDriverJoystick.getLeftTriggerAxis() > 0.8, arm.leftPivot.getPosition());
    claw.run(arm.armPivotPosition());
    drive.run();
    arm.run();
    try {
      // ghloe.run();
    } catch (Exception e) {
      System.out.println(e);
      System.err.println();
    }

  }

  // we hate test mode
  // yes we do
  @Override
  public void testInit() {
    navi.setStartingHeading();
    ghloe.setupCursorChase();
    drive.motorCoastMode();
  }

  @Override
  public void testPeriodic() {

    SmartDashboard.putNumber("Pressure", pneu.getPressure(0));

    boolean wedgeReverse = false;

    if (driverJoystick.getRightBumper()) {
      arm.pivotBrake.set(true); // Brake disengaged
    } else {
      arm.pivotBrake.set(false); // Brake engaged
    }

    if (driverJoystick.getLeftBumper()) {
      arm.stndoBrake.set(true); // Brake disengaged
    } else {
      arm.stndoBrake.set(false); // Brake engaged
    }

    arm.leftPivot.setPercentOutput(driverJoystick.getRightX());
    arm.rightPivot.setPercentOutput(driverJoystick.getRightX());

    arm.retractionMotor.setPercentOutput(-1 * driverJoystick.getLeftY());

    if (driverJoystick.getPOV() == 0) {
      drive.mechanumDrive(0, -0.2, 0);
    } else if (driverJoystick.getPOV() == 90) {
      drive.mechanumDrive(0.2, 0, 0);
    } else if (driverJoystick.getPOV() == 180) {
      drive.mechanumDrive(0, 0.2, 0);
    } else if (driverJoystick.getPOV() == 270) {
      drive.mechanumDrive(-0.2, 0, 0);
    } else {
      drive.mechanumDrive(0, 0, 0);
    }

    if (driverJoystick.getYButtonPressed()) {
      drive.isInTractionDrive = true;
      wedge.deploy(false);
    } else if (driverJoystick.getBButtonPressed()) {
      drive.isInTractionDrive = false;
    }

    if (driverJoystick.getXButton()) {
      claw.weels.setPercentOutput(0.4);
    } else if (driverJoystick.getAButton()) {
      claw.weels.setPercentOutput(-0.4);
    } else {
      claw.weels.setPercentOutput(0);
    }

    if (driverJoystick.getLeftTriggerAxis() > 0.1) {
      claw.wrist.setPercentOutput(0.3 * -driverJoystick.getLeftTriggerAxis());
    } else if (driverJoystick.getRightTriggerAxis() > 0.1) {
      claw.wrist.setPercentOutput(0.3 * driverJoystick.getRightTriggerAxis());
    } else {
      claw.wrist.setPercentOutput(0);
    }

    if (driverJoystick.getStartButton()) {
      wedgeReverse = false;
      wedge.deploy(true);
    } else if (driverJoystick.getBackButton()) {
      wedgeReverse = true;
      wedge.deploy(true);
    } else {
      wedge.deploy(false);
    }

    // ghloe.run();
    drive.run();
    wedge.run(wedgeReverse, 0);

  }

  @Override
  public void teleopExit() {
    drive.motorBrakeMode();
  }

  @Override
  public void testExit() {

  }

  @Override
  public void disabledInit() {
    ghloe.isDisabled = true;
    ghloe.setupCursorChase();
    ghloe.resetVictoryTimer();
  }

  @Override
  public void disabledPeriodic() {

    // ghloe.run();

  }

  @Override
  public void disabledExit() {
    ghloe.isDisabled = false;
  }

  public double robotCurrent() {
    double current = 0;
    for (int i = 0; i < 20; i++) {
      current += pdp.getCurrent(i);
    }
    return current;
  }

}