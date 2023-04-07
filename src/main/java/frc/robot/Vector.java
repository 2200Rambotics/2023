package frc.robot;

public class Vector {
    
    public double x;
    public double y;

    public Vector (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector addVector(Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector multiplyVector(double multiplicand) {
        return new Vector(x * multiplicand, y * multiplicand);
    }

    public String toString() {
        return String.format("(x: %d, y: %d)", x, y);
    }
}
