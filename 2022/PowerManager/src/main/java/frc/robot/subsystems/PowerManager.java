// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.PowerManagerConstants;

public class PowerManager extends SubsystemBase {

  public PowerDistributionPanel m_PDP = new PowerDistributionPanel(PowerManagerConstants.PDP_CAN_ID);
  private static PowerManager instance = new PowerManager();
  double busVoltage, busCurrent, busEnergy, busTemp;
  boolean[] channelUsed = new boolean[16];
  String[] channelName = new String[16];
  boolean[] updateChannel = new boolean[16];
  double[] channelMaxAmp = new double[16];
  double[] channelData = new double[16];

  public PowerManager() {
  
    m_PDP.clearStickyFaults();
    setChannelImmutables();
    updateChannelData();
  }



  public static PowerManager getInstance(){
    return instance; 
  }


  public void updateBusData() {
    busVoltage = m_PDP.getVoltage();
    busCurrent = m_PDP.getTotalCurrent();
    busEnergy = m_PDP.getTotalEnergy();
    busTemp = m_PDP.getTemperature();
    SmartDashboard.putNumber("PDPTotalVolt", busVoltage);
    SmartDashboard.putNumber("PDPTotalCurrent", busCurrent);
    //SmartDashboard.putNumber("PDPTotalEnergy", busEnergy);
    //SmartDashboard.putNumber("PDPTemp", busTemp);
  }

  public void updateChannelData() {
    for (int idx = 0; idx < 16; idx++) {
      if (channelUsed[idx] && updateChannel[idx]) {
        channelData[idx] = m_PDP.getCurrent(idx);
        SmartDashboard.putNumber(channelName[idx], channelData[idx]);
      }
    }
  }

  private void setChannelImmutables() {
    for (int idx = 0; idx < 16; idx++) {
      channelUsed[idx]=PowerManagerConstants.CHANNEL_USED[idx];
      if (channelUsed[idx]) { 
        channelName[idx]=PowerManagerConstants.CHANNEL_NAME[idx]; 
        channelMaxAmp[idx]=PowerManagerConstants.CHANNEL_MAX_CURRENT[idx];
        updateChannel[idx]=PowerManagerConstants.UPDATE_CHANNEL[idx];
      }
    }
  }

  @Override
  public void periodic() {
    updateBusData();
    updateChannelData();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
