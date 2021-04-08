import { Color } from '@/types/color';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { Step } from '@/types/step';
import { RobotConsumption } from '@/types/robotConsumption';
import { PuckList } from '@/types/puckList';

export const defaultState = {
  cycleReady: false,
  cycleStarted: false,
  tableImage: '',
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
  puckColors: [Color.Red, Color.Blue, Color.Green],
  puckFirstCorner: null as Corner | unknown,
  plannedTrajectory: [] as Array<Coordinate>,
  currentPlannedTrajectory: [] as Array<Coordinate>,
  realTrajectory: [] as Array<Coordinate>,
  puckInGrip: false,
  currentStep: Step.CycleNotStarted,
  puckList: new PuckList(),
};

export const state = {
  cycleReady: defaultState.cycleReady,
  cycleStarted: defaultState.cycleStarted,
  tableImage: defaultState.tableImage,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  puckColors: defaultState.puckColors, // TODO : Remove this
  puckFirstCorner: defaultState.puckFirstCorner, // TODO : Remove this
  plannedTrajectory: defaultState.plannedTrajectory,
  currentPlannedTrajectory: defaultState.currentPlannedTrajectory,
  realTrajectory: defaultState.realTrajectory,
  puckInGrip: defaultState.puckInGrip, // TODO : Remove this
  currentStep: defaultState.currentStep,
  puckList: defaultState.puckList,
};

export type State = typeof state;
