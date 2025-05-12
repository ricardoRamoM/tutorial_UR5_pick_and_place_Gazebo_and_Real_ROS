
"use strict";

let AddToLog = require('./AddToLog.js')
let Popup = require('./Popup.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Load = require('./Load.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetProgramState = require('./GetProgramState.js')
let RawRequest = require('./RawRequest.js')
let GetRobotMode = require('./GetRobotMode.js')

module.exports = {
  AddToLog: AddToLog,
  Popup: Popup,
  IsProgramRunning: IsProgramRunning,
  IsInRemoteControl: IsInRemoteControl,
  Load: Load,
  GetLoadedProgram: GetLoadedProgram,
  IsProgramSaved: IsProgramSaved,
  GetSafetyMode: GetSafetyMode,
  GetProgramState: GetProgramState,
  RawRequest: RawRequest,
  GetRobotMode: GetRobotMode,
};
