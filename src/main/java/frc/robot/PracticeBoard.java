package frc.robot;


public class PracticeBoard {
    Motor left;
    Ghloe lights;
    Limelight limelight;
    
    public PracticeBoard(Ghloe lights, Limelight limelight) {
        //left = new Motor(11, "Left Motor", 0, 0, 0, 0, 1);
        //left.init();
        this.lights = lights;
        this.limelight = limelight;
        
    }
    public void printMotors() {
        left.printSensors();
    }
    public void printPIDF() {
        left.printPIDF();
    }
    public void setPIDFDashboard() {
        left.setPIDFDashboard();
    }
    public void set(double power) {
        left.setPercentOutput(power);
    }
    public void setPosition(double position) {
        left.setPosition(position);
    }
    public void resetEncoders() {
        left.resetEncoder();
    }

    //limelight
    public void run() {
        /*
        if (limelight.tv == 1.0) {
            lights.requestCone();
        } else {
            lights.rgbMode();
        }
        */

        /*int lightPosition = (int)(EthanMath.map(limelight.tx, -29.8, 29.8, 0, 15));
        if(limelight.tv == 1.0){
            lights.setColors(lightPosition, new Color(255, 0, 0), new Color(255, 255, 255));
        } else{
            lights.setColors(lightPosition, new Color(0, 0, 0), new Color(0, 0, 0));
        }*/

    }




}
