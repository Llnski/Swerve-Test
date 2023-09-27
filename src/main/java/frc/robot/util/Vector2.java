package frc.robot.util;

public class Vector2 {
    private double x, y;

    public Vector2() {
        this(0, 0);
    }

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector2 plus(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    public Vector2 minus(Vector2 other) {
        return new Vector2(x - other.x, y - other.y);
    }

    public Vector2 times(double scalar) {
        return new Vector2(scalar * x, scalar * y);
    }

    public Vector2 unaryMinus() {
        return this.times(-1);
    }

    public Vector2 div(double scalar) {
        return this.times(1 / scalar);
    }

    public boolean closeTo(Vector2 other, double margin) {
        return minus(other).norm() < margin;
    }

    // Checks if two vectors are within 1e-6 (i.e. effectively equivalent)
    public boolean closeTo(Vector2 other) {
        return closeTo(other, 1e-6);
    }

    public String toString() {
        return "<" + x + ", " + y + ">";
    }

    public double dot(Vector2 other) {
        return x * other.x + y * other.y;
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public Vector2 normalize() {
        return this.div(this.norm());
    }

    public Vector2 cwPerp() {
        return new Vector2(this.y, -this.x);
    }
    // fun rotate(rad: Double) = Vector2(cos(rad) * x - sin(rad) * y, sin(rad) * x +
    // cos(rad) * y)
    // fun rotateDeg(deg: Double) = rotate(deg * PI/180.0)
}
