import { Color } from '@/types/color';
import { RobotConsumption } from '@/types/robotConsumption';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { GripState } from '@/types/gripState';
import { Step } from '@/types/step';

export const defaultState = {
  cycleStarted: false,
  tableImage: '', // TODO : Table image most likely won't be a string, this is temporary
  resistance: 0,
  robotConsumption: {
    batteryPercentage: 0,
    batteryRemainingTimeInSeconds: 0,
  } as RobotConsumption,
  puckColors: [] as Array<Color>,
  puckFirstCorner: null as Corner | unknown,
  plannedTrajectory: [] as Array<Coordinate>,
  realTrajectory: [] as Array<Coordinate>,
  gripState: GripState.released,
  currentStep: Step.CycleNotStarted,
};

export const state = {
  cycleStarted: defaultState.cycleStarted,
  tableImage: defaultState.tableImage,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  puckColors: defaultState.puckColors,
  puckFirstCorner: defaultState.puckFirstCorner,
  plannedTrajectory: defaultState.plannedTrajectory,
  realTrajectory: defaultState.realTrajectory,
  gripState: defaultState.gripState,
  currentStep: defaultState.currentStep,
};

export type State = typeof state;
