package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A controller class for Thrustmaster T-16000M joysticks, PS4 controllers, or Xbox controllers.
 * It wraps the WPILib Joystick methods to make it more user friendly.
 */
public class LazyController {
  private ControllerType type;
  private Joystick controller1;
  private Joystick controller2;

  /**
   * A controller type.
   */
  public enum ControllerType {
    Joysticks,
    PS4,
    Xbox;
  }

  /**
   * Corresponds to the POV or DPAD of the controller.
   */
  public enum Pov {
    UP,
    UP_RIGHT,
    RIGHT,
    DOWN_RIGHT,
    DOWN,
    DOWN_LEFT,
    LEFT,
    UP_LEFT,
    NONE;
  }

  /**
   * Corresponds to buttons on the controller.
   */
  public enum ControllerButton {
    SQUARE(1),
    X(2),
    CIRCLE(3),
    TRIANGLE(4),
    L1(5),
    R1(6),
    L2(7),
    R2(8),
    SHARE(9),
    OPTIONS(10),
    PS4_LEFT_JOYSTICK(11),
    PS4_RIGHT_JOYSTICK(12),
    PLAYSTATION_BUTTON(13),
    TOUCHPAD(14);

    public final int buttonNum;

    private ControllerButton(int buttonNum) {
      this.buttonNum = buttonNum;
    }
  }

  /**
   * Constructs a LazyController with two IDs of a specified type (Only used for Joysticks).
   *
   * @param id1 The ID of the left joystick
   * @param id2 The ID of the right joystick
   * @param type The type of the controller
   */
  public LazyController(int id1, int id2, ControllerType type) {
    this.controller1 = new Joystick(id1);
    this.controller2 = new Joystick(id2);
    this.type = type;
  }

  /**
   * Constructs a LazyController of a specified type.
   *
   * @param id The ID of the controller
   * @param type The type of the controller
   */
  public LazyController(int id, ControllerType type) {
    this.controller1 = new Joystick(id);
    this.type = type;

    // TODO: Log warning if type is Joystick that getLeft methods throw NullPointerException.
  }

  /**
   * Constructs a LazyController with two IDs (Only used for Joysticks).
   *
   * @param id1 The ID of the left joystick
   * @param id2 The ID of the right joystick
   */
  public LazyController(int id1, int id2) {
    this.controller1 = new Joystick(id1);
    this.controller2 = new Joystick(id2);
    this.type = ControllerType.Joysticks;
  }

  /** Constructs a LazyController of type PS4.
   *
   * @param id The ID of the controller
   */
  public LazyController(int id) {
    this.controller1 = new Joystick(id);
    this.type = ControllerType.PS4;
  }

  /**
   * Gets the left joystick X axis.
   *
   * @return The X axis of the left joystick
   */
  public double getLeftX() {
    switch (this.type) {
      case Joysticks:
        return controller1.getRawAxis(0);
      case PS4:
        return controller1.getRawAxis(0);
      default:
        return -1;
    }
  }

  /**
   * Gets the left joystick Y axis.
   *
   * @return The Y axis of the left joystick
   */
  public double getLeftY() {
    switch (this.type) {
      case Joysticks:
        return -controller1.getRawAxis(1);
      case PS4:
        return -controller1.getRawAxis(1);
      default:
        return -1;
    }
  }

  /**
   * Gets the left joystick Z axis.
   *
   * @return The Z axis of the left joystick
   */
  public double getLeftZ() {
    switch (this.type) {
      case Joysticks:
        return controller1.getRawAxis(2);
      default:
        return -1;
    }
  }

  /**
   * Gets the left joystick slider.
   *
   * @return The slider value of the left joystick
   */
  public double getLeftSlider() {
    switch (this.type) {
      case Joysticks:
        return -controller1.getRawAxis(2);
      default:
        return -1;
    }
  }

  /**
   * Gets the left joystick Pov.
   *
   * @return The Pov of the left joystick
   */
  public Pov getLeftPov() {
    return convertToPov(controller1.getPOV());
  }

  /**
   * Gets whether the specified button is pressed on the left controller.
   *
   * @return Whether buttonNum is pressed
   */
  public boolean getLeftButton(int buttonNum) {
    return controller1.getRawButton(buttonNum);
  }

  /**
   * Gets whether the specified button is pressed on the left controller.
   *
   * @return Whether button is pressed
   */
  public boolean getLeftButton(ControllerButton button) {
    return getLeftButton(button.buttonNum);
  }

  /**
   * Gets the right joystick X axis.
   *
   * @return The X axis of the right joystick
   */
  public double getRightX() {
    switch (this.type) {
      case Joysticks:
        return controller2.getRawAxis(0);
      case PS4:
        return controller1.getRawAxis(2);
      default:
        return -1;
    }
  }

  /**
   * Gets the right joystick Y axis.
   *
   * @return The Y axis of the right joystick
   */
  public double getRightY() {
    switch (this.type) {
      case Joysticks:
        return -controller2.getRawAxis(1);
      case PS4:
        return -controller1.getRawAxis(5);
      default:
        return -1;
    }
  }

  /**
   * Gets the right joystick Z axis.
   *
   * @return The Z axis of the right joystick
   */
  public double getRightZ() {
    switch (this.type) {
      case Joysticks:
        return controller2.getRawAxis(2);
      default:
        return -1;
    }
  }

  /**
   * Gets the right joystick slider.
   *
   * @return The slider value of the right joystick
   */
  public double getRightSlider() {
    switch (this.type) {
      case Joysticks:
        return -controller2.getRawAxis(3);
      default:
        return -1;
    }
  }

  /**
   * Gets the right joystick Pov.
   *
   * @return The Pov of the right joystick
   */
  public Pov getRightPov() {
    switch (this.type) {
      case Joysticks:
        return convertToPov(controller2.getPOV());
      case PS4:
        return convertToPov(controller1.getPOV());
      default:
        return Pov.NONE;
    }
  }

  /**
   * Gets whether the specified button is pressed on the right controller.
   *
   * @return Whether buttonNum is pressed
   */
  public boolean getRightButton(int buttonNum) {
    switch (this.type) {
      case Joysticks:
        return controller2.getRawButton(buttonNum);
      case PS4:
        return controller1.getRawButton(buttonNum);
      default:
        return false;
    }
  }

  /**
   * Gets whether the specified button is pressed on the right controller.
   *
   * @return Whether button is pressed
   */
  public boolean getRightButton(ControllerButton button) {
    return getRightButton(button.buttonNum);
  }

  /**
   * Converts a number representing a Pov position to a Pov.
   *
   * @param povNum A number representing a Pov position
   * @return The Pov the number represented
   */
  private Pov convertToPov(int povNum) {
    switch (povNum) {
      case 0:
        return Pov.UP;
      case 45:
        return Pov.UP_RIGHT;
      case 90:
        return Pov.RIGHT;
      case 135:
        return Pov.DOWN_RIGHT;
      case 180:
        return Pov.DOWN;
      case 225:
        return Pov.DOWN_LEFT;
      case 270:
        return Pov.LEFT;
      case 315:
        return Pov.UP_LEFT;
      default:
        return Pov.NONE;
    }
  }
}
