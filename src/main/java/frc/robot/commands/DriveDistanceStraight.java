package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceStraight extends DriveDistance {
  /* The original gyro heading of the robot */
  private double initialHeading = 0.0;

  /**
   * Proportional term of heading correction controller
   *
   * <p>This value is a fraction of the total output power. This means that it will likely be quite
   * low.
   */
  private static final double TURN_KP = 0.07;

  /** Integrated term of heading correction controller */
  private static final double TURN_KI = 0.0005;

  /** Derivative term of heading correction controller */
  private static final double TURN_KD = 0.0;

  private PIDController headingController = new PIDController(TURN_KP, TURN_KI, TURN_KD);

  /**
   * Creates a new DriveDistanceStraight. This command will drive your your robot for a desired
   * distance at a desired speed, while trying to correct for any turning that may occur
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistanceStraight(double speed, double inches, Drivetrain drive) {
    super(speed, inches, drive);
  }

  @Override
  public void initialize() {
    headingController.reset();
    initialHeading = m_drive.getGyroAngleZ();
    super.initialize();
  }

  @Override
  public void execute() {
    double currentHeading = m_drive.getGyroAngleZ();
    double headingError = currentHeading - initialHeading;
    double headingCorrection = headingController.calculate(headingError);

    SmartDashboard.putNumber("DriveDistanceStraight/speed", m_speed);
    SmartDashboard.putNumber("DriveDistanceStraight/headingError", headingError);
    SmartDashboard.putNumber("DriveDistanceStraight/headingCorrection", headingCorrection);
    m_drive.arcadeDrive(m_speed, headingCorrection, false);
  }
}
