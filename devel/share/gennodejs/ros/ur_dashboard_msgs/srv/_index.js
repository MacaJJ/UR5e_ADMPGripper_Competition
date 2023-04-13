
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let Popup = require('./Popup.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetProgramState = require('./GetProgramState.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let AddToLog = require('./AddToLog.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let RawRequest = require('./RawRequest.js')
let Load = require('./Load.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  GetLoadedProgram: GetLoadedProgram,
  Popup: Popup,
  IsProgramRunning: IsProgramRunning,
  IsProgramSaved: IsProgramSaved,
  GetProgramState: GetProgramState,
  IsInRemoteControl: IsInRemoteControl,
  AddToLog: AddToLog,
  GetSafetyMode: GetSafetyMode,
  RawRequest: RawRequest,
  Load: Load,
};
