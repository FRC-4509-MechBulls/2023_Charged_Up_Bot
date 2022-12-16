// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  //CAN Config Variables
    private static int configIndex;
    private static boolean configDone;
  //motors
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;
  //abs encoder
    private final DutyCycleEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
  //Values
    private double delta = 0;
    private double deltaConverted = 0;
    private double setAngle = 0;
    private SwerveModuleState state;
  //Dashboard
    //Tabs
      private ShuffleboardTab tabModules;
      private ShuffleboardTab tabModulePID;
    //Entries
      //Values
        //abs encoders
          private NetworkTableEntry dashboardAbsRaw01;
      //Status
        //abs encoders
          private NetworkTableEntry dashboardAbsConnected;
      //PID
        //drive
          private NetworkTableEntry dashboardkAFF;
          private NetworkTableEntry dashboardkFDrive;
          private NetworkTableEntry dashboardkPDrive;
          private NetworkTableEntry dashboardErrorDrive;
          private NetworkTableEntry dashboardSetpointDrive;
        //Turn
          private NetworkTableEntry dashboardkPTurn;
          private NetworkTableEntry dashboardErrorTurn;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //Config Index
      configIndex = 0;
      configDone = false;
    //Dashboard
      tabModules = Shuffleboard.getTab("Modules");
      tabModulePID = Shuffleboard.getTab("ModulePID");
    //absolute encoder
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
      absoluteEncoder.setDistancePerRotation(1);
    //motors
      //drive
      driveMotor = new WPI_TalonFX(driveMotorId);
      driveMotor.setInverted(driveMotorReversed);
      //turn
      turningMotor = new WPI_TalonFX(turningMotorId);
      turningMotor.setInverted(turningMotorReversed);
      //both
    //Debug
      debugInit();
  }

  //Configuration
    public void config (int index) {
      if (index==0){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 3001, 5);return;}
      if (index==1){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 3003, 5);return;}
      if (index==2){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 3007, 5);return;}
      if (index==3){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 3011, 5);return;}
      if (index==4){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 3013, 5);return;}
      if (index==5){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 3017, 5);return;}
      if (index==6){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 3021, 5);return;}
      if (index==7){driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 3023, 5);return;}
      if (index==8){driveMotor.configFactoryDefault(5);return;}
      if (index==9){driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveMotor, 5);return;}
      if (index==10){driveMotor.setNeutralMode(NeutralMode.Coast);return;}

      if (index==11){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 3001, 5);return;}
      if (index==12){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 3003, 5);return;}
      if (index==13){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 3007, 5);return;}
      if (index==14){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 3011, 5);return;}
      if (index==15){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 3013, 5);return;}
      if (index==16){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 3017, 5);return;}
      if (index==17){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 3021, 5);return;}
      if (index==18){turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 3023, 5);return;}
      if (index==19){turningMotor.configFactoryDefault(5);return;}
      if (index==20){turningMotor.configAllSettings(Robot.ctreConfigs.swerveTurnMotor, 5);return;}
      if (index==21){turningMotor.setNeutralMode(NeutralMode.Coast);return;}
      
      if (index==22){enableVoltageCompensation(true);return;}
      if (index==23){enableOverrideLimitSwitches(true);return;}
      if (index==24){resetEncoders();configDone=true;return;}
    }
    public void resetEncoders() {
      driveMotor.setSelectedSensorPosition(0); //reset drive motor encoder to 0
      turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad() * ModuleConstants.kRadiansToTurning); //resets turning motor encoder to absolute encoder value
    }
    public void enableVoltageCompensation(boolean onOff) {
      driveMotor.enableVoltageCompensation(onOff);
      turningMotor.enableVoltageCompensation(onOff);
    }
    public void enableOverrideLimitSwitches(boolean onOff) {
      driveMotor.overrideLimitSwitchesEnable(onOff);
      driveMotor.overrideSoftLimitsEnable(onOff);
      turningMotor.overrideLimitSwitchesEnable(onOff);
      turningMotor.overrideSoftLimitsEnable(onOff);
    }
  //Getters
    public double getTurningPosition() {
      return 0;//turningMotor.getSelectedSensorPosition() / ModuleConstants.kRadiansToTurning; removed
    }
    public double getDriveVelocity() {
      return 0;//driveMotor.getSelectedSensorVelocity() / ModuleConstants.kMetersToDriveVelocity; //convert raw sensor units to m/s removed
    }
    public double getAbsoluteEncoderRad() {
      double angle = absoluteEncoder.getAbsolutePosition(); //range 0-1
      //Debug output: 
      tabModules.add("absRadians" + absoluteEncoder.getSourceChannel(), angle);
      angle *= ModuleConstants.kAbsToRadians; //converts to radians
      angle += absoluteEncoderOffsetRad; //subtracts the offset to get the actual wheel angles
      return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //multiply -1 if reversed
    }
    public SwerveModuleState getState() { //wpi lib requests info in form of swerve module state, so this method converts it
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
  //Setters
    public void setDesiredState(SwerveModuleState state) {
      this.state = state;
      if (Math.abs(this.state.speedMetersPerSecond) < 0.0001) { //prevents wheels from going to OG pos when joysticks are not moved
        driveMotor.set(TalonFXControlMode.Velocity, 0);
        return;
      }
      calculateFalconRelativeState();
      /*
      driveMotor.set(TalonFXControlMode.Velocity, this.state.speedMetersPerSecond * ModuleConstants.kMetersToDriveVelocity, 
                     DemandType.ArbitraryFeedForward, (this.state.speedMetersPerSecond/Math.abs(this.state.speedMetersPerSecond)) * ModuleConstants.kAFFDrive); //velocity control
      turningMotor.set(TalonFXControlMode.Position, setAngle * ModuleConstants.kRadiansToTurning); //Position Control
      removed
      */
    }
    public void stop() { //sets both voltage outputs to 0
      /*
      driveMotor.set(TalonFXControlMode.PercentOutput, 0);
      turningMotor.set(TalonFXControlMode.PercentOutput, 0);
      removed
      */
    }
  //Utility
    private void calculateFalconRelativeState() { //converts absolute state to falcon-encoder-relative state
      state = SwerveModuleState.optimize(state, getState().angle); //makes it so wheel never turns more than 90 deg
      delta = state.angle.getRadians() - getTurningPosition(); //turn error
      deltaConverted = delta % Math.PI; //error converted to representative of the actual gap; error > pi indicates we aren't taking the shortest route to setpoint, but rather doing one or more 180* rotations.this is caused by the discontinuity of numbers(pi is the same location as -pi, yet -pi is less than pi)
      setAngle = Math.abs(deltaConverted) < (Math.PI / 2) ? getTurningPosition() + deltaConverted : 
                                                            getTurningPosition() - ((deltaConverted/Math.abs(deltaConverted)) * (Math.PI-Math.abs(deltaConverted))); //makes set angle +/- 1/2pi of our current position(capable of pointing all directions)
    }
  //Dashboard
  //Debugging
    //PID
      public void debugTuneModulePIDInit() { //call from debugInit()
        //Drive
          dashboardkAFF = tabModulePID.add("kAFFDrive", ModuleConstants.kAFFDrive).getEntry();
          dashboardkFDrive = tabModulePID.add("kFDrive", ModuleConstants.kFDrive).getEntry();
          dashboardkPDrive = tabModulePID.add("kPDrive", ModuleConstants.kPDrive).getEntry();
          dashboardErrorDrive = tabModulePID.add("ErrorDrive" + absoluteEncoder.getSourceChannel(), driveMotor.getClosedLoopError()).getEntry();
          dashboardSetpointDrive = tabModulePID.add("SetpointDrive" + absoluteEncoder.getSourceChannel(), 
                                                    driveMotor.getClosedLoopTarget() / ModuleConstants.kMetersToDriveVelocity).getEntry();
        //Turn
          dashboardkPTurn = tabModulePID.add("kPTurn", ModuleConstants.kPTurning).getEntry();
          dashboardErrorTurn = tabModulePID.add("errorT" + absoluteEncoder.getSourceChannel(), turningMotor.getClosedLoopError()).getEntry();
        }
      public void debugTuneModulePIDSetDesiredState() { //call from setDesiredState()
        //Drive
          dashboardErrorDrive.setDouble(driveMotor.getClosedLoopError());
          dashboardSetpointDrive.setDouble(driveMotor.getClosedLoopTarget() / ModuleConstants.kMetersToDriveVelocity);
          driveMotor.config_kP(0, dashboardkPDrive.getDouble(ModuleConstants.kPDrive));
          driveMotor.config_kF(0, dashboardkFDrive.getDouble(ModuleConstants.kFDrive));
          turningMotor.set(TalonFXControlMode.Position, setAngle * ModuleConstants.kRadiansToTurning); //Position Control
        //Turn
          dashboardErrorTurn.setDouble(turningMotor.getClosedLoopError());
          turningMotor.config_kP(0, dashboardkPTurn.getDouble(ModuleConstants.kPTurning));
          turningMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond * ModuleConstants.kMetersToDriveVelocity, 
                           DemandType.ArbitraryFeedForward, (state.speedMetersPerSecond/Math.abs(state.speedMetersPerSecond)) * dashboardkAFF.getDouble(ModuleConstants.kAFFDrive));
      }
    //Other
      public void debugInit() {
        //Encoders
          dashboardAbsConnected = tabModules.add("absConnected" + absoluteEncoder.getSourceChannel(), absoluteEncoder.isConnected()).getEntry();
          //Debug output: dashboardAbsRaw01 = tabModules.add("abs0-1" + absoluteEncoder.getSourceChannel(), absoluteEncoder.getAbsolutePosition()).getEntry();
        //debugTuneModulePIDInit();
      }
      public void debugPeriodic() {
        //Encoders
          dashboardAbsConnected.setBoolean(absoluteEncoder.isConnected());
          //Debug output: tabModules.add("absRadians" + absoluteEncoder.getSourceChannel(), getAbsoluteEncoderRad());
          //Debug output: dashboardAbsRaw01.setDouble(absoluteEncoder.getAbsolutePosition());
        //Math
          //Debug output: tabModules.add("deltaC" + absoluteEncoder.getSourceChannel(), deltaConverted);
        //Output
          //Debug output: tabModules.add("Voltd" + absoluteEncoder.getSourceChannel(), driveMotor.getMotorOutputVoltage());
          //Debug output: tabModules.add("stateAngle" + absoluteEncoder.getSourceChannel(), getState().angle.getRadians());
          //Debug output: tabModules.add("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString());
          //Debug output: tabModules.add("setRadians" + absoluteEncoder.getSourceChannel(), state.angle.getRadians());
      }

    @Override
  public void periodic() { // This method will be called once per scheduler run
    if (!configDone) {
      config(configIndex);
      configIndex++;
    } 
    //Debug
      debugPeriodic();
  }
}
