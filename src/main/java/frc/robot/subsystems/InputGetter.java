// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DSSim;
import frc.robot.util.InputInterface;
import frc.robot.util.InputInterface.Inputs;

public class InputGetter extends SubsystemBase {
	private Inputs inputs;
    private DSSim dsSim;
    private boolean lastEnableStatus = false;

    public InputGetter() {
        if(!Constants.IS_MASTER) {
            dsSim = new DSSim();
            dsSim.init();
        }
    }

    @Override
    public void periodic() {
    	inputs = InputInterface.grabInputs();

        //only do this logic if we aren't the master. 
        if(!Constants.IS_MASTER) {
            // if the enable state has changed, we need to update the DSSim
            if(inputs.isEnabled != lastEnableStatus) {
                if(inputs.isEnabled) {
                    dsSim.enable();
                } else {
                    dsSim.disable();
                }

                lastEnableStatus = inputs.isEnabled;
            }

            System.out.println(dsSim.isEnabled);
        }

        //debugging/testing
	}

    public DoubleSupplier getLeftX() {
        return () -> inputs.leftX;
    }
    public DoubleSupplier getLeftY() {
        return () -> inputs.leftY;
    }
    public DoubleSupplier getRightX() {
        return () -> inputs.rightX;
    }
    public DoubleSupplier getRightY() {
        return () -> inputs.rightY;
    }
    public BooleanSupplier getAButton() {
        return () -> inputs.aButton;
    }
    public BooleanSupplier getBButton() {
        return () -> inputs.bButton;
    }
    public BooleanSupplier getXButton() {
        return () -> inputs.xButton;
    }
    public BooleanSupplier getYButton() {
        return () -> inputs.yButton;
    }
    public BooleanSupplier getLeftBumper() {
        return () -> inputs.leftBumper;
    }
    public BooleanSupplier getRightBumper() {
        return () -> inputs.rightBumper;
    }
    public BooleanSupplier getStartButton() {
        return () -> inputs.startButton;
    }
    public BooleanSupplier getBackButton() {
        return () -> inputs.backButton;
    }
}
