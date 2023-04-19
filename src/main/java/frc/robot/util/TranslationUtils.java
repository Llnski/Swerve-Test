package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class TranslationUtils {
    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static Translation2d normalize(Translation2d p) {
        return p.div(p.getNorm());
    }

    public static Translation2d cwPerp(Translation2d vec) {
        return new Translation2d(vec.getY(), -vec.getX());
    }
}
