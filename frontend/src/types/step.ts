export enum Step {
  CycleReady = "cycleReady", //waiting mode, all booted up
  CycleStarted = "cycleStarted", //display all robot comsumtion at this moment
  ToResistanceStation = "toResistanceStation",
  CalculateResistance = "calculateResistance",
  ToControlPanel = "toControlPanel",
  DecodeControlPanel = "decodeControlPanel",
  ToFirstPuck = "toFirstPuck",
  GrabFirstPuck = "grabFirstPuck",
  toFirstCorner = "toFirstCorner",
  ToSecondPuck = "toSecondPuck",
  GrabSeoncdPuck = "grabFirstPuck",
  toSecondCorner = "toSecondCorner",
  ToLastPuck = "toLastPuck",
  GrabLastPuck = "grabLastPuck",
  ToLastCorner = "toLastCorner",
  FinalRobotPlacement = "finalRobotPlacement", //into green square
  CycleEnded = 'cycleEnded',//RED LED to show its finished
}
