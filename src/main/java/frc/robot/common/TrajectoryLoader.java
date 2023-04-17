package frc.robot.common;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class TrajectoryLoader {

  private static final String OUTPUT_DIRECTORY_NAME = "output";

  public TrajectoryLoader() {}

  private List<File> getTrajectoryFiles() {
    File pathsDirectory =
        Filesystem.getDeployDirectory().toPath().resolve(OUTPUT_DIRECTORY_NAME).toFile();
    File[] trajectoryFiles = pathsDirectory.listFiles();

    return Arrays.asList(trajectoryFiles);
  }

  public HashMap<String, Trajectory> loadTrajectories() {
    HashMap<String, Trajectory> trajectories = new HashMap<>();
    List<File> files = getTrajectoryFiles();
    for (File file : files) {
      Trajectory trajectory;
      try {
        trajectory = TrajectoryUtil.fromPathweaverJson(file.toPath());
        trajectories.put(file.getName().split("\\.")[0], trajectory);
      } catch (IOException ex) {
        return trajectories;
      }
    }
    System.out.println("FOUND " + trajectories.size() + " TRAJECTORIES");
    return trajectories;
  }
}
