package frc.robot.sensors.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface EstimatorModuleIO {
    public static final class PoseEstimate {
        public Pose2d estimatedPose;
        public Matrix<N3, N1> stdDevs;
        public double timestamp;

        public PoseEstimate(Pose2d estimatedPose, Matrix<N3, N1> stdDevs) {
            this.estimatedPose = estimatedPose;
            this.stdDevs = stdDevs;
            this.timestamp = Logger.getInstance().getTimestamp() / 1.0e6;
        }
    }

    @AutoLog
    public static class EstimatorInputs {
        public double timestamp, latency, ambiguity, targetCount;
        public double bestTgtArea, bestTgtID, bestTgtAmbiguity, bestTgtDist;

        public double lastPoseTimestamp;
        public double[] stdDevs = new double[0];
        public double[] estPose = new double[0];
        public double[] estTgtCornersX = new double[0];
        public double[] estTgtCornersY = new double[0];

        public boolean isConnected;
    }

    public boolean updateInputs(EstimatorInputsAutoLogged inputs);

    public void setCamera(EstimatorCamera camera);

    public Optional<PoseEstimate> getPoseEstimation(Pose2d currentEstimatedPose);
}
