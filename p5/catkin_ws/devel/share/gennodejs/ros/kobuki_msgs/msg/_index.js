
"use strict";

let BumperEvent = require('./BumperEvent.js');
let MotorPower = require('./MotorPower.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let ButtonEvent = require('./ButtonEvent.js');
let ControllerInfo = require('./ControllerInfo.js');
let KeyboardInput = require('./KeyboardInput.js');
let DigitalOutput = require('./DigitalOutput.js');
let ScanAngle = require('./ScanAngle.js');
let VersionInfo = require('./VersionInfo.js');
let DockInfraRed = require('./DockInfraRed.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let SensorState = require('./SensorState.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let Sound = require('./Sound.js');
let ExternalPower = require('./ExternalPower.js');
let CliffEvent = require('./CliffEvent.js');
let Led = require('./Led.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');

module.exports = {
  BumperEvent: BumperEvent,
  MotorPower: MotorPower,
  PowerSystemEvent: PowerSystemEvent,
  ButtonEvent: ButtonEvent,
  ControllerInfo: ControllerInfo,
  KeyboardInput: KeyboardInput,
  DigitalOutput: DigitalOutput,
  ScanAngle: ScanAngle,
  VersionInfo: VersionInfo,
  DockInfraRed: DockInfraRed,
  DigitalInputEvent: DigitalInputEvent,
  SensorState: SensorState,
  WheelDropEvent: WheelDropEvent,
  RobotStateEvent: RobotStateEvent,
  Sound: Sound,
  ExternalPower: ExternalPower,
  CliffEvent: CliffEvent,
  Led: Led,
  AutoDockingActionResult: AutoDockingActionResult,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingResult: AutoDockingResult,
  AutoDockingAction: AutoDockingAction,
  AutoDockingFeedback: AutoDockingFeedback,
};
