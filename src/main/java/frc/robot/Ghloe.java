package frc.robot;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * DO NOT set all of the LEDs to white!
 */
public class Ghloe implements Runnable {
    // ghloing :)

    /**
     *
     */
    private static final Color BLUE_GLW = new Color(0, 0, 75);
    private static final Color RED_GLOW = new Color(75, 0, 0);
    private static final Color COLOR_BLUE = new Color(0, 0, 255);
    private static final Color COLOR_GREEN = new Color(0, 255, 0);
    private static final Color REQUEST_CUBE_COLOR_ON = new Color(255, 0, 255);
    private static final Color REQUEST_CONE_COLOR_ON = new Color(255, 175, 0);
    private static final Color FALLING_OVER_OFF_COLOR = new Color(0, 0, 0);
    private static final Color FALLING_OVER_ON_COLOR = new Color(255, 0, 127);

    AddressableLED ledStrip;

    final AddressableLEDBuffer onBuffer;
    final AddressableLEDBuffer offBuffer;
    AddressableLEDBuffer showingBuffer;

    final AddressableLEDBuffer currentBuffer;
    final AddressableLEDBuffer voltageBuffer;
    final AddressableLEDBuffer pressureBuffer;

    Timer flashingTimer;
    Timer timer;

    int rainbowCounter = 0;

    // Which Robot?
    boolean isCompetitionRobot = true;

    public boolean isRainbow = false;
    public boolean isDisabled = false;
    public boolean isFallingOver = false;
    public boolean isBalancing = false;
    public boolean isRequestingCone = false;
    public boolean isRequestingCube = false;
    public boolean isShowingCurrent = false;
    public boolean hasPiece = false;
    public boolean isBrakeMode = false;

    int position;

    // PowerDistribution pdp;
    PneumaticHub pneu;
    NavX navi;
    Robot robot;

    final int length;
    final Strip lfStrip;
    final Strip lbStrip;
    final Strip rfStrip;
    final Strip rbStrip;

    final Strip leftStrip;
    final Strip rightStrip;

    final Strip fullStrip;

    Timer victoryTimer;

    public synchronized void resetVictoryTimer() {
        victoryTimer.restart();
    }

    public synchronized double getVictoryTimer() {
        return victoryTimer.get();
    }

    private class Strip {
        public final int start;
        public final int end; // end is one past the end of the strip
        public final int direction;
        public final int length;

        /**
         * @param start: start index of LED strip
         * @param end:   one past the last LED index
         */
        public Strip(int start, int end) {

            this.start = start;
            this.end = end;

            length = Math.abs(start - end);

            if (start < end) {
                direction = 1;
            } else {
                direction = -1;
            }

        }
    }

    public Ghloe(int pwmPort, PneumaticHub pneu, NavX navi, Robot robot) {
        ledStrip = new AddressableLED(pwmPort);
        position = 0;

        if (isCompetitionRobot) {
            lfStrip = new Strip(0, 15);
            lbStrip = new Strip(29, 14);
            rfStrip = new Strip(30, 45);
            rbStrip = new Strip(59, 44);

            leftStrip = new Strip(0, 30);
            rightStrip = new Strip(30, 60);

            fullStrip = new Strip(0, 60);

            length = 60;
        } else {
            lfStrip = new Strip(0, 14);
            lbStrip = new Strip(27, 13);
            rfStrip = new Strip(28, 42);
            rbStrip = new Strip(55, 41);

            leftStrip = new Strip(0, 28);
            rightStrip = new Strip(28, 56);

            fullStrip = new Strip(0, 56);
            length = 56;
        }

        victoryTimer = new Timer();
        victoryTimer.start();

        onBuffer = new AddressableLEDBuffer(length);
        offBuffer = new AddressableLEDBuffer(length);
        showingBuffer = new AddressableLEDBuffer(length);
        currentBuffer = new AddressableLEDBuffer(length);
        voltageBuffer = new AddressableLEDBuffer(length);
        pressureBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(onBuffer.getLength());

        ledStrip.setData(onBuffer);
        ledStrip.start();

        timer = new Timer();
        flashingTimer = new Timer();
        this.pneu = pneu;
        this.navi = navi;
        this.robot = robot;

        timer.start();
        flashingTimer.start();
    }

    public void run() {
        while (true) {
            realRun();
            try {
                if (getVictoryTimer() < 1.0) {
                    Thread.sleep(1);
                } else {
                    Thread.sleep(20);
                }
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void realRun() {
        if (isDisabled) { // Disco!!!
            setupCursorChase();
            // setColour(Color.kBeige, onBuffer, fullStrip);
            showingBuffer = onBuffer;
        } else if (hasPiece) {
            setColour(Color.kWhite, onBuffer, fullStrip);
            setColour(Color.kBlack, offBuffer, fullStrip);
            flashing(onBuffer, offBuffer, 0.1);
        } else if (isBrakeMode) { // Alternating Orange and Black
            showingBuffer = requestBrakeMode();
        } else if (isBalancing) { // White (3) and Black(off) and Yellow (Alternating)
            showingBuffer = requestBalancing();
        } else if (isFallingOver) { // Pink
            showingBuffer = requestImFallingOverOhNoOhNo();
        } else if (isRequestingCone) { // Yellow
            showingBuffer = requestCone();
        } else if (isRequestingCube) { // Purple
            showingBuffer = requestCube();
        } else if (isShowingCurrent) { // Red
            showingBuffer = requestCurrent();
        } else { // Battery (Green) and Pressure (Blue)
            showingBuffer = requestVoltage(onBuffer);
            requestPressure(onBuffer);
            showingBuffer = onBuffer;
        }

        ledStrip.setData(showingBuffer);
        position++;
    }

    public AddressableLEDBuffer flashing(AddressableLEDBuffer onBuffer, AddressableLEDBuffer offBuffer, double period) {
        if ((flashingTimer.get() % period) > (period / 2.0)) {
            return onBuffer;
        } else {
            return offBuffer;
        }
    }

    private AddressableLEDBuffer requestImFallingOverOhNoOhNo() {
        setColour(FALLING_OVER_ON_COLOR, onBuffer, fullStrip);
        setColour(FALLING_OVER_OFF_COLOR, offBuffer, fullStrip);

        return flashing(onBuffer, offBuffer, 0.05);
    }

    private AddressableLEDBuffer requestBalancing() {
        double pitch = navi.getPitch();
        Color bgColor = Color.kViolet;
        int position = (int) EthanMath.map(pitch, -15, 15, 0, lbStrip.length);
        if (pitch < 5 && pitch > -5) {
            bgColor = Color.kGreen;
        } else {
            if (DriverStation.getAlliance() == Alliance.Red) {
                bgColor = Color.kDarkRed;
            } else {
                bgColor = Color.kDarkBlue;
            }
        }
        setColour(bgColor, onBuffer, fullStrip);
        drawCursor(onBuffer, position, Color.kWhite, lfStrip);
        drawCursor(onBuffer, position, Color.kWhite, lbStrip);
        drawCursor(onBuffer, position, Color.kWhite, rfStrip);
        drawCursor(onBuffer, position, Color.kWhite, rbStrip);

        return onBuffer;
    }

    private AddressableLEDBuffer requestBrakeMode() {
        alternating(onBuffer, Color.kOrange, Color.kBlack);
        return onBuffer;
    }

    private AddressableLEDBuffer requestCone() {
        setColour(REQUEST_CONE_COLOR_ON, onBuffer, fullStrip);
        setColour(Color.kBlack, offBuffer, fullStrip);
        return flashing(onBuffer, offBuffer, 0.4);
    }

    private AddressableLEDBuffer requestCube() {
        setColour(REQUEST_CUBE_COLOR_ON, onBuffer, fullStrip);
        setColour(Color.kBlack, offBuffer, fullStrip);
        return flashing(onBuffer, offBuffer, 0.4);
    }

    private AddressableLEDBuffer requestCurrent() {
        double current = robot.robotCurrent();
        int onLEDs = (int) EthanMath.map(current, 0, 100, 0, currentBuffer.getLength());

        return currentBuffer;
    }

    private AddressableLEDBuffer requestVoltage(AddressableLEDBuffer buffer) {
        double voltage = robot.pdp.getVoltage();
        int onLEDs = (int) EthanMath.map(voltage, 10, 12.5, 0, lbStrip.length);
        SmartDashboard.putNumber("voltage", voltage);

        setOnOff(buffer, onLEDs, COLOR_GREEN, Color.kBlack, lfStrip);
        setOnOff(buffer, onLEDs, COLOR_GREEN, Color.kBlack, rbStrip);
        return buffer;
    }

    private AddressableLEDBuffer requestPressure(AddressableLEDBuffer buffer) {
        double pressure = pneu.getPressure(0);

        int onLEDs = (int) EthanMath.map(pressure, 50, 120, 0, lfStrip.length);

        setOnOff(buffer, onLEDs, COLOR_BLUE, FALLING_OVER_OFF_COLOR, lbStrip);
        setOnOff(buffer, onLEDs, COLOR_BLUE, FALLING_OVER_OFF_COLOR, rfStrip);
        return buffer;
    }

    private AddressableLEDBuffer setOnOff(AddressableLEDBuffer buffer, int onLEDLength, Color onColor, Color offColor,
            Strip strip) {
        int onLEDs = 0;

        if (onLEDLength <= 0) {
            onLEDLength = 0;
        }
        if (onLEDLength > strip.length) {
            onLEDLength = strip.length;
        }
        if (strip.direction == 1) {
            if (strip.start + onLEDLength > strip.end) {
                onLEDLength = strip.end - strip.start;
            }
            for (int i = strip.start; onLEDs < onLEDLength; i++, onLEDs++) {
                buffer.setLED(i, onColor);
            }
            for (int i = strip.start + onLEDLength; i < strip.end; i++) {
                buffer.setLED(i, offColor);
            }
        } else {
            if (strip.start - onLEDLength < strip.end) {
                onLEDLength = strip.start - strip.end;
            }
            for (int i = strip.start; onLEDs < onLEDLength; i += strip.direction, onLEDs++) {
                buffer.setLED(i, onColor);
            }
            for (int i = strip.start + (onLEDLength * strip.direction); i > strip.end; i += strip.direction) {
                buffer.setLED(i, offColor);
            }
        }

        return buffer;
    }

    public AddressableLEDBuffer setColour(Color color, AddressableLEDBuffer buffer, Strip strip) {
        for (var i = strip.start; i != strip.end; i += strip.direction) {
            // Sets the specified LED to the RGB values for yellow
            buffer.setLED(i, color);
        }
        return buffer;
    }

    public AddressableLEDBuffer drawCursor(AddressableLEDBuffer buffer, int position, Color color, Strip strip) {
        int realposition = strip.start + (position * strip.direction);
        int ib = realposition - strip.direction; // LED Before Index
        int i = realposition; // LED Index
        int ia = realposition + strip.direction; // LED After Index

        if (strip.direction > 0) { // normal case
            if (ib < strip.start || ib >= strip.end) {

            } else {
                buffer.setLED(ib, color);
            }

            if (i < strip.start || i >= strip.end) {

            } else {
                buffer.setLED(i, color);
            }

            if (ia < strip.start || ia >= strip.end) {

            } else {
                buffer.setLED(ia, color);
            }

        } else { // reverse case
            if (ib > strip.start || ib <= strip.end) {

            } else {
                buffer.setLED(ib, color);
            }

            if (i > strip.start || i <= strip.end) {

            } else {
                buffer.setLED(i, color);
            }

            if (ia > strip.start || ia <= strip.end) {

            } else {
                buffer.setLED(ia, color);
            }
        }

        return buffer;
    }

    public AddressableLEDBuffer alternating(AddressableLEDBuffer buffer, Color color1, Color color2) {
        boolean colorpicker = true;
        for (int i = 0; i < buffer.getLength(); i++) {
            if (colorpicker) {
                buffer.setLED(i, color1);
                colorpicker = false;
            } else {
                buffer.setLED(i, color2);
                colorpicker = true;
            }
        }
        return buffer;
    }

    // public AddressableLEDBuffer discoMode(AddressableLEDBuffer buffer) {
    // if (timer.get() > 0.1) {
    // for (var i = 0; i < buffer.getLength(); i++) {
    // // Sets the specified LED to the RGB values for purple
    // int r = ((int) (Math.random() * 2)) * 255;
    // int g = ((int) (Math.random() * 2)) * 255;
    // int b = ((int) (Math.random() * 2)) * 255;
    // if (r + g + b == 0 || r + g + b == 255 * 3) {
    // discoMode();
    // } else {
    // ledBuffer.setRGB(i, r, g, b);
    // }
    // }

    // timer.reset();
    // }
    // }

    public void chaseMode(AddressableLEDBuffer ledBuffer, Strip strip) {

        Color temp = ledBuffer.getLED(strip.start);
        for (int i = strip.start; i != strip.end; i += strip.direction) {
            if (!(i == strip.end - strip.direction)) {
                ledBuffer.setLED(i, ledBuffer.getLED(i + 1));
            }
        }
        ledBuffer.setLED(strip.end - strip.direction, temp);
    }

    // public void stripes(Color foregroundColor, Color backgroundColor, int size) {
    // int counter = 0;
    // boolean colorpicker = true;
    // Color color = foregroundColor;
    // for (int i = 0; i < ledBuffer.getLength(); i++) {
    // if (counter == size) {
    // if (colorpicker){
    // color = backgroundColor;
    // colorpicker = false;
    // } else {
    // color = foregroundColor;
    // colorpicker = true;
    // }
    // counter = 0;
    // }
    // ledBuffer.setLED(i, color);
    // counter++;
    // }
    // }

    public void setupCursorChase() {
        int ledPosition = position % leftStrip.length;

        if (DriverStation.getAlliance() == Alliance.Red) {
            setColour(RED_GLOW, onBuffer, leftStrip);
            setColour(RED_GLOW, onBuffer, rightStrip);

            drawCursor(onBuffer, ledPosition, Color.kViolet, leftStrip);
            drawCursor(onBuffer, ledPosition, Color.kViolet, rightStrip);
        } else {
            setColour(BLUE_GLW, onBuffer, leftStrip);
            setColour(BLUE_GLW, onBuffer, rightStrip);

            drawCursor(onBuffer, ledPosition, Color.kViolet, leftStrip);
            drawCursor(onBuffer, ledPosition, Color.kViolet, rightStrip);
        }
    }

}