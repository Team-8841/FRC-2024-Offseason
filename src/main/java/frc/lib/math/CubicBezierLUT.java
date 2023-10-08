package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;

public class CubicBezierLUT {
    double lutVals[][];

    /**
     * Creates a lookup table from the values of a lookup table.
     * 
     * @param lutVals A 2d array where the first element represents t=0 and the last
     *                represents t=1. lutVals[x][0] is the estimated arclength since
     *                t=0. (lutVals[x][1], lutVals[x][2]) are the (x, y) coordinates
     *                of the point.
     */
    public CubicBezierLUT(double lutVals[][]) {
        this.lutVals = lutVals;
    }

    public static double invLerp(double initial, double end, double interpolated) {
        return (interpolated - initial) / (end - initial);
    }

    public static double lerp(double initial, double end, double t) {
        return (1 - t) * initial + t * end;
    }

    public double getDistAtT(double t) {
        if (t <= 0) {
            return this.lutVals[0][0];
        }

        int lowerIndex = (int) (t * (this.lutVals.length - 1));
        int upperIndex = lowerIndex + 1;

        if (upperIndex >= this.lutVals.length) {
            return this.lutVals[lowerIndex][0];
        }

        double lowerArc = this.lutVals[lowerIndex][0];
        double upperArc = this.lutVals[upperIndex][1];

        double u = invLerp(lowerIndex, upperIndex, t * (this.lutVals.length - 1));

        return lerp(lowerArc, upperArc, u);
    }

    // TODO: Maybe replace this with a binary search?
    public double getTAtDist(double arclength) {
        for (int i = 1; i < this.lutVals.length; i++) {
            if (this.lutVals[i][0] > arclength) {
                // Interpolate between the two found entries
                double t1 = (double) (i - 1) / (this.lutVals.length - 1);
                double t2 = (double) i / (this.lutVals.length - 1);

                double s1 = this.lutVals[i - 1][0];
                double s2 = this.lutVals[i][0];

                double u = invLerp(s1, s2, arclength);

                return lerp(t1, t2, u);
            }
        }

        return 1;
    }

    private double distSqFromIndex(int i, Translation2d pos) {
        double dx = this.lutVals[i][1] - pos.getX();
        double dy = this.lutVals[i][2] - pos.getY();

        return dx * dx + dy * dy;
    }

    public double getClosestT(Translation2d pos) {
        int closestInd = 0;
        double closestDist = this.distSqFromIndex(0, pos);

        for (int i = 1; i < this.lutVals.length; i++) {
            double dist = this.distSqFromIndex(i, pos);

            if (dist < closestDist) {
                closestInd = i;
                closestDist = dist;
            }
        }
        
        return (double) closestInd / (this.lutVals.length - 1);
    }
}
