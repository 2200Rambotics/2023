package frc.robot.auto;

import java.io.File;
import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.Filesystem;
// ImplementAuto - Import Auto (Quick Fix on the type when creating the auto)
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm;
import frc.robot.Robot;
import frc.robot.auto.autos.MovingArm;
import frc.robot.auto.autos.TestAuto;
import frc.robot.auto.autos.TrajectoryAuto;
import frc.robot.auto.commands.Strafe;
import frc.robot.auto.infrastructure.Command;
import frc.robot.auto.autos.StrafeTest;

/**
 * Decides which auto will run
 * Use CTRL-SHIFT-F and search 'ImplementAuto' for everything needed to
 * implement an autonomous
 */
public class Autonomous {

    // ImplementAuto - Add in a name for the autonomous
    private static final String defaultAutoLabel = "Default";
    private static final String testAutoLabel = "Test";
    private static final String strafeTestLabel = "Strafe Test";
    private static final String movingArmLabel = "Moving Arm";

    // ImplementAuto - Create auto object
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    TestAuto testAuto;
    StrafeTest testStrafe;
    MovingArm movingArm;

    Robot robot;

    HashMap<String, Command> autoCommandMap = new HashMap<>();

    public Autonomous(Robot robot) {
        // ImplementAuto - Add in an option for the chooser, use string for both
        // parameters

        m_chooser.setDefaultOption(defaultAutoLabel, defaultAutoLabel);
    
        m_chooser.addOption(testAutoLabel, testAutoLabel);
        m_chooser.addOption(strafeTestLabel, strafeTestLabel);
        m_chooser.addOption(movingArmLabel, movingArmLabel);

        // TODO discover all "trajectory"-style autos and add them to the chooser
        System.out.println("################################################################################");
        System.out.println("################################################################################");
        try {
            File folder = new File(Filesystem.getDeployDirectory(), "pathplanner/");
            File[] listOfFiles = folder.listFiles();

            for (var fileObj : listOfFiles) {
                String fileName = fileObj.getName();
                if (fileObj.isFile() && fileName.contains(".path")) {
                    
                    String pattern = ".+?(?:v([0-9]+\\.[0-9]+) a([0-9]+\\.[0-9]+)).*\\.path";
                    
                    // Create a Pattern object
                    Pattern r = Pattern.compile(pattern);
                    
                    // Now create matcher object.
                    Matcher m = r.matcher(fileName);

                    double maxVelocity = 4.0;
                    double maxAccel = 2.3;
                    
                    if (m.find()) {
                        System.out.println("group count: " + m.groupCount());
                        for (int i = 0; i <= m.groupCount(); i++ ) {
                            System.out.println("Found value: " + m.group(i));
                        }
                        maxVelocity = Double.parseDouble(m.group(1));
                        maxAccel = Double.parseDouble(m.group(2));
                    } else {
                        System.out.println("NO MATCH");
                    }
                    
                    // Get the file name without extension. Path planner lib requires this format
                    int pos = fileName.lastIndexOf(".");
                    if (pos > 0 && pos < (fileName.length() - 1)) { // If '.' is not the first or last character.
                        fileName = fileName.substring(0, pos);
                    }

                    // Create Trajectory Auto object for this trajectory file, and add it to the map
                    TrajectoryAuto auto = new TrajectoryAuto(robot.drive, robot.arm, robot.claw, robot.navi, fileName,
                            maxVelocity, maxAccel);
                    autoCommandMap.put(fileName, auto);
                    m_chooser.addOption(fileName, fileName);
                    System.out.println("File " + fileName);
                }
            }
        } catch (Exception e) {

        }
        System.out.println("################################################################################");
        System.out.println("################################################################################");

        SmartDashboard.putData("Auto choices", m_chooser);

        this.robot = robot;

        // ImplementAuto - Point the autonomous object to a new one

        testAuto = new TestAuto(robot.drive, robot.arm, robot.claw, robot.navi);
        testStrafe = new StrafeTest(robot.drive);
        movingArm = new MovingArm(robot.arm);
    }

    public void Init() {
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);
    }

    public void Periodic() {
        // ImplementAuto - Add in a case, with the string as the case name, and call the
        // run() function
        switch (m_autoSelected) {
            case defaultAutoLabel:
            default:
                // We didn't find any of the static autos, let's check to see if we have a
                // trajectory auto
                // in the hashmap to run
                if (autoCommandMap.containsKey(m_autoSelected)) {
                    Command auto = autoCommandMap.get(m_autoSelected);
                    auto.run();
                }
                break;
            // Test Autos
            case testAutoLabel:
                testAuto.run();
                break;
            case strafeTestLabel:
                testStrafe.run();
                break;
            case movingArmLabel:
                movingArm.run();
                break;
        }

        robot.arm.run();
        robot.drive.run();
        robot.claw.run(robot.arm.armPivotPosition());
        // robot.ghloe.run();
        robot.limelight.run();

    }

}
