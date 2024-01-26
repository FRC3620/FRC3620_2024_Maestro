// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Add your docs here. */
public class FieldLayout {
     static private AprilTagFieldLayout aprilTag2024FieldLayout = null;

    public static AprilTagFieldLayout getAprilTag2023FieldLayout() throws IOException {
        if (aprilTag2024FieldLayout == null) {
            aprilTag2024FieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        return aprilTag2024FieldLayout;
    }
}
