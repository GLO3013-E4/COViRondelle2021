import { MutationTree } from 'vuex/types';
import {
  START_CYCLE,
  SOCKET_ROBOT_CONSUMPTION,
  SOCKET_BATTERY_CONSUMPTION,
  SOCKET_TABLE_IMAGE,
  SOCKET_RESISTANCE,
  SOCKET_PUCK_COLORS,
  SOCKET_PUCK_FIRST_CORNER,
  SOCKET_PLANNED_TRAJECTORY_COORDINATES,
  SOCKET_REAL_TRAJECTORY_COORDINATE,
  SOCKET_GRIP_STATE,
  SOCKET_CURRENT_STEP,
  SOCKET_CYCLE_READY,
} from './mutation-types';
import { defaultState, State } from './state';
import { Message } from '@/types/message';
import { Step } from '@/types/step';

export type Mutations<S = State> = {
  [START_CYCLE](state: S): void;
  [SOCKET_CYCLE_READY](state: S): void;
  [SOCKET_ROBOT_CONSUMPTION](state: S, data: string): void;
  [SOCKET_BATTERY_CONSUMPTION](state: S, data: string): void;
  [SOCKET_RESISTANCE](state: S, data: string): void;
  [SOCKET_PUCK_COLORS](state: S, data: string): void;
  [SOCKET_PUCK_FIRST_CORNER](state: S, data: string): void;
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: S, data: string): void;
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: S, data: string): void;
  [SOCKET_GRIP_STATE](state: S, data: string): void;
  [SOCKET_CURRENT_STEP](state: S, data: string): void;
};

const toMessage = (data: string): Message => JSON.parse(data);

export const mutations: MutationTree<State> & Mutations = {
  [START_CYCLE](state: State) {
    state.currentStep = Step.CycleStarted;
  },
  [SOCKET_CYCLE_READY](state: State) {
    state.cycleReady = true;
  },
  [SOCKET_ROBOT_CONSUMPTION](state: State, data: string) {
    const message = toMessage(data);
    state.robotConsumption =
      message.robotConsumption || defaultState.robotConsumption;
  },
  [SOCKET_BATTERY_CONSUMPTION](state: State, data: string) {
    const message = toMessage(data);
    state.batteryConsumption =
      message.batteryConsumption || defaultState.batteryConsumption;
  },
  [SOCKET_TABLE_IMAGE](state: State, data: string) {
    const message = toMessage(data);
    state.tableImage = message.tableImage || defaultState.tableImage;
  },
  [SOCKET_RESISTANCE](state: State, data: string) {
    const message = toMessage(data);
    state.resistance = message.resistance || defaultState.resistance;
  },
  [SOCKET_PUCK_COLORS](state: State, data: string) {
    const message = toMessage(data);
    state.puckColors = message.puckColors || defaultState.puckColors;
  },
  [SOCKET_PUCK_FIRST_CORNER](state: State, data: string) {
    const message = toMessage(data);

    state.puckFirstCorner =
      message.puckFirstCorner || defaultState.puckFirstCorner;
  },
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: State, data: string) {
    const message = toMessage(data);
    if (message.plannedTrajectoryCoordinates)
      state.plannedTrajectory.push(...message.plannedTrajectoryCoordinates);
  },
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: State, data: string) {
    const message = toMessage(data);
    if (message.realTrajectoryCoordinate)
      state.realTrajectory.push(message.realTrajectoryCoordinate);
  },
  [SOCKET_GRIP_STATE](state: State, data: string) {
    const message = toMessage(data);
    state.puckInGrip = message.puckInGrip || defaultState.puckInGrip;
  },
  [SOCKET_CURRENT_STEP](state: State, data: string) {
    const message = toMessage(data);
    state.currentStep = message.currentStep || defaultState.currentStep;
  },
};
