// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.util.LimelightHelpers.LimelightResults;

/** Packages useful information for AprilTag localization into an object created from json results */
public class CamData {
  public final Pose2d pose2d;
  public final Pose3d pose3d;
  public final double latency, ta, avgTagDist;
  public final int numTargets, pipeline;
  public final boolean valid;
  public final String name;

  public CamData(String name) {
    this.name = name;
    pipeline = (int) LimelightHelpers.getCurrentPipelineIndex(name);
    LimelightResults results;
    if (LimelightHelpers.getTV(name)) {
      results = LimelightHelpers.getLatestResults(name);
      valid = results.targetingResults.valid;
    } else {
      valid = false;
      results = null;
    }
    if (valid) {
      pose3d = new Pose3d(
        new Translation3d(
          results.targetingResults.botpose_wpiblue[0],
          results.targetingResults.botpose_wpiblue[1],
          results.targetingResults.botpose_wpiblue[2]
        ),
        new Rotation3d(
          Math.toRadians(results.targetingResults.botpose_wpiblue[3]),
          Math.toRadians(results.targetingResults.botpose_wpiblue[4]),
          Math.toRadians(results.targetingResults.botpose_wpiblue[5])
        )
      );
      pose2d = pose3d.toPose2d();
      latency = 
        results.targetingResults.latency_capture
        + results.targetingResults.latency_pipeline
        + results.targetingResults.latency_jsonParse;
      avgTagDist = results.targetingResults.botpose_avgdist;
      ta = results.targetingResults.botpose_avgarea;
      numTargets = (int) results.targetingResults.botpose_tagcount;
    } else {
      pose3d = new Pose3d();
      pose2d = new Pose2d();
      latency = 0;
      ta = 0;
      avgTagDist = 0;
      numTargets = 0;
    }
  }
}
