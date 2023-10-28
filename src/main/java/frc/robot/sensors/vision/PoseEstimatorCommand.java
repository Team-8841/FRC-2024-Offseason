package frc.robot.sensors.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimatorCommand extends CommandBase {
    EstimatorCamera camera;
    EstimatorModuleIO estimatorModule;
    SwerveDrivePoseEstimator poseEstimator;
    EstimatorInputsAutoLogged loggedInputs;

    public PoseEstimatorCommand(EstimatorCamera camera, EstimatorModuleIO estimatorModule, SwerveDrivePoseEstimator poseEstimator) {
        this.addRequirements(camera);

        this.camera = camera;
        this.estimatorModule = estimatorModule;
        this.poseEstimator = poseEstimator;
        this.loggedInputs = new EstimatorInputsAutoLogged();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.estimatorModule.setCamera(this.camera);
    }

    @Override
    public void execute() {
        Optional<EstimatorModuleIO.PoseEstimate> poseEstimateOpt = this.estimatorModule
                .getPoseEstimation(this.poseEstimator.getEstimatedPosition());

        if (this.estimatorModule.updateInputs(this.loggedInputs)) {
            Logger.getInstance().processInputs(String.format("/Vision/%s/inputs", this.camera.name), this.loggedInputs);
        }

        poseEstimateOpt.ifPresent((poseEstimate) -> {
            double timestamp = Logger.getInstance().getTimestamp() / 1000000.0;
            this.poseEstimator.addVisionMeasurement(poseEstimate.estimatedPose, timestamp, poseEstimate.stdDevs);
        });
    }
}
