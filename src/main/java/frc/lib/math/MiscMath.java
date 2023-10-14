package frc.lib.math;

public class MiscMath {
    public static double invLerp(double initial, double end, double interpolated) {
        return (interpolated - initial) / (end - initial);
    }

    public static double lerp(double initial, double end, double t) {
        return (1 - t) * initial + t * end;
    }
}
