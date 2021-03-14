import { Color } from '@/types/color';
import { BatteryConsumption } from '@/types/batteryConsumption';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { Step } from '@/types/step';
import { RobotConsumption } from '@/types/robotConsumption';

export const defaultState = {
  cycleReady: false,
  cycleStarted: false,
  tableImage: '', // TODO : Table image most likely won't be a string, this is temporary
  resistance: 0,
  batteryConsumption: {
    batteryChargeLeft: 8,
    batteryRemainingTimeInSeconds: 0,
  } as BatteryConsumption,
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    servoMotor: 0,
    total: 0,
  } as RobotConsumption,
  puckColors: [] as Array<Color>,
  puckFirstCorner: null as Corner | unknown,
  plannedTrajectory: [] as Array<Coordinate>,
  realTrajectory: [] as Array<Coordinate>,
  puckInGrip: false,
  currentStep: Step.CycleNotStarted,
};

export const state = {
  cycleReady: defaultState.cycleReady,
  cycleStarted: defaultState.cycleStarted,
  tableImage: defaultState.tableImage,
  resistance: defaultState.resistance,
  batteryConsumption: defaultState.batteryConsumption,
  robotConsumption: defaultState.robotConsumption,
  puckColors: defaultState.puckColors,
  puckFirstCorner: defaultState.puckFirstCorner,
  plannedTrajectory: defaultState.plannedTrajectory,
  realTrajectory: defaultState.realTrajectory,
  puckInGrip: defaultState.puckInGrip,
  currentStep: defaultState.currentStep,
};

export type State = typeof state;
