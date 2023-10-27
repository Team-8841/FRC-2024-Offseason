package frc.robot.constants;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
    public static final AprilTagFieldLayout fieldLayout;
    public static final double minimumTagArea = 0.2;

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(1, 1, 2);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.7);

    static {
        try {
            fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        }
        catch(IOException e) {
            throw new RuntimeException(e);
        }
    }
}
