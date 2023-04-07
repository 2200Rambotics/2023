package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Wedge {
    DoubleSolenoid frontWedgeSol;
    DoubleSolenoid backWedgeSol;

    boolean frontDeployed;
    boolean backDeployed;
    boolean wedgeDeployRequest;

    double frontArmDeadband = 10;
    double backArmDeadband = -10;

    Timer wedgeFrontTimer;
    Timer wedgeBackTimer;

    public Wedge(){
        frontWedgeSol = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.FRONT_WEDGE_FWD_SOL_ID, Constants.FRONT_WEDGE_REV_SOL_ID );
        backWedgeSol = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.BACK_WEDGE_FWD_SOL_ID, Constants.BACK_WEDGE_REV_SOL_ID);
        
        frontDeployed = false;
        backDeployed = false;
        wedgeDeployRequest = false;

        wedgeFrontTimer = new Timer();
        wedgeBackTimer = new Timer();

        wedgeFrontTimer.start();
        wedgeBackTimer.start();
    }

    private void frontWedgeUP(){
        frontWedgeSol.set(Value.kReverse);
    }

    private void frontWedgeDown(){
        frontWedgeSol.set(Value.kForward);
    }

    private void backWedgeUP(){
        backWedgeSol.set(Value.kReverse);
    }

    private void backWedgeDown(){
        backWedgeSol.set(Value.kForward);
    }

    // Changes boolean request which is used in the arm.run function
    public void deploy(boolean request){
        wedgeDeployRequest = request;
    }

    public void run(boolean isReverse, double armPosition){

        // if Requested
        if(wedgeDeployRequest){
            // Arm position is outside of the deadband (backwards) = back wedges go down
            if(isReverse){
                frontDeployed = false;
                backDeployed = true;
            } else { // Arm position is in the deadband = no wedges deployed
                frontDeployed = true;
                backDeployed = false;
            }
        }
        else{ // No request made = no wedges deployed
            frontDeployed = false;
            backDeployed = false;
        }

        // Preventing both wedges from being down at the same time
        if(frontDeployed && wedgeBackTimer.get() > 0.5 && armPosition > backArmDeadband){ // Front wedge down, back wedge up
            frontWedgeDown();
            backWedgeUP();
            wedgeFrontTimer.reset();
        }
        else if(backDeployed && wedgeFrontTimer.get() > 0.5 && armPosition < frontArmDeadband){ // Front wedge up, back wedge down
            backWedgeDown();
            frontWedgeUP();
            wedgeBackTimer.reset();
        }
        else{ // both wedges up
            frontWedgeUP();
            backWedgeUP();
        }
    }

    /*
     * Used on WedgeControl for autonomous
     */
    public void wedgeAuto(boolean isDown, boolean isFront) {
        // If wedge down is requested, checks which side, and puts them down. Puts both up if requested up
        if(isDown) {
            if(isFront) {
                frontWedgeDown();
            } else {
                backWedgeDown();
            }
        } else {
            frontWedgeUP();
            backWedgeUP();
        }
    }

    public void printStatus(){
        SmartDashboard.putBoolean("Wedge Deploy Request", wedgeDeployRequest);
        SmartDashboard.putBoolean("Back Wedge Deployed", backDeployed);
        SmartDashboard.putBoolean("Front Wedge Deployed", frontDeployed);
    }
}
