import { Color } from '@/types/color';
import { RobotConsumption } from '@/types/robotConsumption';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { GripState } from '@/types/gripState';

export const defaultState = {
  cycleStarted: false,
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
};

export const state = {
  cycleStarted: defaultState.cycleStarted,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  puckColors: defaultState.puckColors,
  puckFirstCorner: defaultState.puckFirstCorner,
  plannedTrajectory: defaultState.plannedTrajectory,
  realTrajectory: defaultState.realTrajectory,
  gripState: defaultState.gripState,
  // TODO : Implement other state values
};

export type State = typeof state;
