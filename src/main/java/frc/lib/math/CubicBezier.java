package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Cubic bezier curve
 * @see <a href="https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves">Cubic Bezier Curve Wikipedia</a>
 * @see <a href="https://www.youtube.com/watch?v=aVwxzDHniEw">Freya's video on Bezier curves</a>
 * @see <a href="https://www.youtube.com/watch?v=jvPPXbo87ds">Freya's video on splines</a>
 */
public class CubicBezier {
    private Translation2d points[];

    public static class LUT {
        double lutVals[][];

        /**
         * Creates a lookup table from the values of a lookup table.
         * 
         * @param lutVals A 2d array where the first element represents t=0 and the last
         *                represents t=1. lutVals[x][0] is the estimated arclength since
         *                t=0. (lutVals[x][1], lutVals[x][2]) are the (x, y) coordinates
         *                of the point.
         */
        public LUT(double lutVals[][]) {
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

    /**
     * @param points The control points.
     */
    public CubicBezier(Translation2d points[]) {
        if (points.length != 4) {
            throw new IllegalArgumentException("Illegal amount of points.");
        }

        this.points = points;
    }

    public LUT getLUT() {
        return null;
    }

    /**
     * Calculates the point of the curve at paramter t.
     * @param t
     * @return The point of the curve at parameter t.
     */
    public Translation2d get(double t) {
        Translation2d a = this.points[0].times((1 - t) * (1 - t) * (1 - t));
        Translation2d b = this.points[1].times(3 * (1 - t) * (1 - t) * t);
        Translation2d c = this.points[2].times(3 * (1 - t) * t * t);
        Translation2d d = this.points[3].times(t * t * t);

        return a.plus(b).plus(c).plus(d);
    }

    /**
     * Calculates the derivative of the curve at parameter t.
     * @param t
     * @return The second derivative at parameter t.
     */
    public Translation2d getDerivative(double t) {
        Translation2d a = this.points[1].minus(this.points[0]).times(3 * (1 - t) * (1 - t));
        Translation2d b = this.points[2].minus(this.points[1]).times(6 * (1 - t) * t);
        Translation2d c = this.points[3].minus(this.points[2]).times(3 * t * t);

        return a.plus(b).plus(c);
    }

    /**
     * Calculates the second derivative of the curve at parameter t.
     * @param t
     * @return The second derivative at parameter t.
     */
    public Translation2d getSecondDerivative(double t) {
        Translation2d a = this.points[2].minus(this.points[1].times(2)).plus(this.points[0]);
        a = a.times(6 * (1 - t));

        Translation2d b = this.points[3].minus(this.points[2].times(2)).plus(this.points[1]);
        b = b.times(6 * t);

        return a.plus(b);
    }

    /**
     * Calculates the curvature at parameter t.
     * @see <a href="https://en.wikipedia.org/wiki/Curvature">Curvature Wikipedia</a>
     * @param t
     * @return The curvature at parameter t
     */
    public double getCurvature(double t) {
        // third equation on https://en.wikipedia.org/wiki/Curvature#General_expressions 
        Translation2d derivative = this.getDerivative(t);
        Translation2d secondDerivative = this.getSecondDerivative(t);

        double derivativeMagSq = derivative.getX() * derivative.getX() + derivative.getY() * derivative.getY();
        double secondDerivativeMagSq = secondDerivative.getX() * secondDerivative.getX()
                + secondDerivative.getY() * secondDerivative.getY();

        double upperDotProd = derivative.getX() * secondDerivative.getX() + derivative.getY() * secondDerivative.getY();

        double upper = Math.sqrt(
                derivativeMagSq * secondDerivativeMagSq
                        - upperDotProd * upperDotProd);
        double lower = derivativeMagSq * Math.sqrt(derivativeMagSq);

        return upper / lower;
    }
}
