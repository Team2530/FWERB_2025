// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;


import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightContainer {
  static int SIMCOUNTER = 0;
  static int RLCOUNTER = 0;
  static int RLCountermt1 = 0;
  private static ArrayList<Limelight> limelights = new ArrayList<Limelight>();

  public LimelightContainer(Limelight... limelights) {
    for (Limelight limelight : limelights) {
      LimelightContainer.limelights.add(limelight);
    }
    enableLimelights(true);
    SmartDashboard.putString("Limelights added and enabled", "true");
  }

  public void enableLimelights(boolean enable) {
    for (Limelight limelight : limelights) {
      limelight.setEnabled(enable);
    }
  }
  public static void estimateSimOdometry(){
    SmartDashboard.putString("Estimating sim", "yes");
    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
      if(mt2==null){ // in case not all limelights are connected
        continue;
      }
      SmartDashboard.putString("Not null: ", limelight.toString() + " " + limelight.getName() + " \n"+ mt2.toString());
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        SmartDashboard.putString("Simulated Pos", mt2.pose.toString()+SIMCOUNTER);
        SIMCOUNTER++;
      }
    }
  }

  public void estimateMT2Odometry(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds speeds, AHRS navx) {
    SmartDashboard.putBoolean("Updating Odometry: ", false);

    for (Limelight limelight : limelights) {
      boolean doRejectUpdate = false;
      boolean doRejectUpdateMT1 = false;
      

      LimelightHelpers.SetRobotOrientation(limelight.getName(),
      poseEstimator.getEstimatedPosition().getRotation().getDegrees(), navx.getRate(), 0, 0, 33, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());

      if(mt1==null || mt1.tagCount == 0){
        doRejectUpdateMT1 = true;
      }

      if(mt2==null){
        continue;
      }
      if (Math.abs(navx.getRate()) > 720) {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        SmartDashboard.putBoolean("Updating Odometry: ", true);
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      } 
      SmartDashboard.putString("Pos", mt2.pose.toString()+" "+ RLCOUNTER);
      SmartDashboard.putString("Pos MT1: ", mt1.pose.toString()+" " + RLCountermt1);
      RLCOUNTER++;  
      RLCountermt1++;
    }
  }

}
