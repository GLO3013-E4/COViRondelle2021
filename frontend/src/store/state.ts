import { Color } from '@/types/color';
import { RobotConsumption } from '@/types/robotConsumption';

export const defaultState = {
  cycleStarted: false,
  resistance: 0,
  robotConsumption: {
    batteryPercentage: 0,
    batteryRemainingTimeInSeconds: 0,
  } as RobotConsumption,
  puckColors: [] as Array<Color>,
};

export const state = {
  cycleStarted: defaultState.cycleStarted,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  puckColors: defaultState.puckColors,
  // TODO : Implement other state values
};

export type State = typeof state;
