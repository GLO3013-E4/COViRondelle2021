import { Color } from '@/types/color';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { Step } from '@/types/step';
import { RobotConsumption } from '@/types/robotConsumption';

export const defaultState = {
  cycleReady: false,
  cycleStarted: false,
  tableImage: '/stub_table_image.jpg', // TODO : Table image most likely won't be a string, this is temporary
  resistance: 0,

  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 0,
  } as RobotConsumption,
  puckColors: [] as Array<Color>,
  puckFirstCorner: null as Corner | unknown,
  plannedTrajectory: [] as Array<Coordinate>,
  currentPlannedTrajectory: [] as Array<Coordinate>,
  realTrajectory: [] as Array<Coordinate>,
  puckInGrip: false,
  currentStep: Step.CycleNotStarted,
};

export const state = {
  cycleReady: defaultState.cycleReady,
  cycleStarted: defaultState.cycleStarted,
  tableImage: defaultState.tableImage,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  puckColors: defaultState.puckColors,
  puckFirstCorner: defaultState.puckFirstCorner,
  plannedTrajectory: defaultState.plannedTrajectory,
  currentPlannedTrajectory: defaultState.currentPlannedTrajectory,
  realTrajectory: defaultState.realTrajectory,
  puckInGrip: defaultState.puckInGrip,
  currentStep: defaultState.currentStep,
};

export type State = typeof state;
