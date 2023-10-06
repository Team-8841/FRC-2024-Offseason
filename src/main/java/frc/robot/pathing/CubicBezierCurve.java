package frc.robot.pathing;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Cubic bezier curve
 * @see <a href="https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves">Cubic Bezier Curve Wikipedia</a>
 * @see <a href="https://www.youtube.com/watch?v=aVwxzDHniEw">Freya's video on Bezier curves</a>
 * @see <a href="https://www.youtube.com/watch?v=jvPPXbo87ds">Freya's video on splines</a>
 */
public class CubicBezierCurve {
    private Translation2d points[];

    /**
     * @param points The control points.
     */
    public CubicBezierCurve(Translation2d points[]) {
        if (points.length != 4) {
            throw new IllegalArgumentException("Illegal amount of points.");
        }

        this.points = points;
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
