// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

/** Add your docs here. */
public class MotorSetup {
    boolean inverted = false;
    boolean coast = false;
    Integer currentLimit = null;
    static private int defaultCurrentLimit = 10;

    public MotorSetup setInverted (boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MotorSetup setCoast (boolean coast) {
        this.coast = coast;
        return this;
    }

    public MotorSetup setCurrentLimit (int currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public void apply(CANSparkMax x) {
        x.restoreFactoryDefaults();
        x.setInverted(inverted);
        x.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        x.setOpenLoopRampRate(1);
        x.setClosedLoopRampRate(1);
        x.setSmartCurrentLimit(currentLimit == null ? defaultCurrentLimit : currentLimit);
    }
    
    public void apply(TalonFX x) {
        int kTimeoutMs = 0;
        var talonFXConfiguration = new TalonFXConfiguration();

        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorOutputConfigs.NeutralMode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        var openLoopRamps = new OpenLoopRampsConfigs();
        openLoopRamps.VoltageOpenLoopRampPeriod = 1;
        talonFXConfiguration.OpenLoopRamps = openLoopRamps;

        var closedLoopRamps = new ClosedLoopRampsConfigs();
        closedLoopRamps.VoltageClosedLoopRampPeriod = 1;
        talonFXConfiguration.ClosedLoopRamps = closedLoopRamps;

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit((currentLimit == null) ? defaultCurrentLimit : currentLimit).withStatorCurrentLimitEnable(true);
        talonFXConfiguration.CurrentLimits = currentLimitsConfigs;

        x.getConfigurator().apply(talonFXConfiguration);
    }
    
}