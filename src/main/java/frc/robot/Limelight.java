package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class grabs data and values from the
 * limelight and prints them to the smartdashboard.
 */
public class Limelight {
    // Limelight Documentation
    // https://docs.limelightvision.io/en/latest/networktables_api.html
    NetworkTable centerLimelightVals;
    NetworkTable redLimelightVals;
    NetworkTable blueLimelightVals;

    LimeState lState;

    /**
     * Range: -29.8 to 29.8.
     * tx is our offset from the middle of the camera to the target on the x-axis
     */
    public double tx;

    /**
     * Range: -24.85 to 24.85
     * ty is our offset from the middle of the camera to the target on the y-axis
     */
    public double ty;

    /**
     * Returns 0 or 1.0.
     * Does our robot see any valid targets?
     */
    public double tv;

    /**
     * Returns 0% to 100%
     * Target area as a percentage of the image
     */
    public double ta;

    /**
     * Range: 0 to 9
     * Gets the current pipeline
     */
    public double getpipe;

    public double sideTA;
    public double sideTX;
    public double sideTV;

    DoubleLogEntry txLogEntry;
    DoubleLogEntry tyLogEntry;
    DoubleLogEntry tvLogEntry;
    DoubleLogEntry taLogEntry;

    Object valueMutex = new Object();

    public Limelight() {
        // Intializes the network tables on the limelight so we are able to grab
        // limelight values
        centerLimelightVals = NetworkTableInstance.getDefault().getTable("limelight-forward");
        redLimelightVals = NetworkTableInstance.getDefault().getTable("limelight-red");
        blueLimelightVals = NetworkTableInstance.getDefault().getTable("limelight-blue");

        try {
            txLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "tx");
            tyLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "ty");
            taLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "tv");
            tvLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "ta");

        } catch (Exception e) {
            // TODO: handle exception
        }

    }

    public void run() {
        // myLockObject.lock()
        // do code
        // myLockObject.unLock()
        synchronized (valueMutex) {
            // Updates limelight values, runs constantly
            tx = printLimes("center", "tx", centerLimelightVals);
            ty = printLimes("center", "ty", centerLimelightVals);
            tv = printLimes("center", "tv", centerLimelightVals);
            ta = printLimes("center", "ta", centerLimelightVals);
            getpipe = printLimes("center", "getpipe", centerLimelightVals);

            double redTA = printLimes("red", "ta", redLimelightVals);
            double redTX = printLimes("red", "tx", redLimelightVals);
            double redTV = printLimes("red", "tv", redLimelightVals);
            double blueTA = printLimes("blue", "ta", blueLimelightVals);
            double blueTX = printLimes("blue", "tx", blueLimelightVals);
            double blueTV = printLimes("blue", "tv", blueLimelightVals);

            if (DriverStation.getAlliance() == Alliance.Red) {
                sideTA = redTA;
                sideTX = redTX;
                sideTV = redTV;
            } else {
                sideTA = blueTA;
                sideTX = blueTX;
                sideTV = blueTV;
            }

            try {
                txLogEntry.append(tx);
                tyLogEntry.append(ty);
                taLogEntry.append(ta);
                tvLogEntry.append(tv);
            } catch (Exception e) {
                // TODO: handle exception
            }
        }
    }

    public double printLimes(String prefix, String key, NetworkTable table) {
        // Prints to smartdashboard and returns values
        double limelightvalue = table.getEntry(key).getDouble(0);
        SmartDashboard.putNumber(prefix + " " + key, limelightvalue);
        return limelightvalue;
    }

    public void setPipeline(LimeState ligState) {
        lState = ligState;
        if (ligState == LimeState.OFF) {
            centerLimelightVals.getEntry("pipeline").setNumber(0);
        } else if (ligState == LimeState.GREEN) {
            centerLimelightVals.getEntry("pipeline").setNumber(1);
        } else if (ligState == LimeState.FEEDER) {
        } else {
            centerLimelightVals.getEntry("pipeline").setNumber(2);
        }

    }

    public LimeState getlState() {
        synchronized (valueMutex) {
            return lState;
        }
    }

    public double getTx() {
        synchronized (valueMutex) {

            return tx;
        }
    }

    public double getTy() {
        synchronized (valueMutex) {
            return ty;
        }
    }

    public double getTv() {
        synchronized (valueMutex) {
            return tv;
        }
    }

    public double getTa() {
        synchronized (valueMutex) {
            return ta;
        }
    }

    public double getGetpipe() {
        synchronized (valueMutex) {
            return getpipe;
        }
    }

    public double getSideTA() {
        synchronized (valueMutex) {
            return sideTA;
        }
    }

    public double getSideTX() {
        synchronized (valueMutex) {
            return sideTX;
        }
    }

    public double getSideTV() {
        synchronized (valueMutex) {
            return sideTV;
        }
    }

}
