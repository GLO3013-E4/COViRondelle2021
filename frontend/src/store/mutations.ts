import { MutationTree } from 'vuex/types';
import {
  SOCKET_CURRENT_STEP,
  SOCKET_GRIP_STATE,
  SOCKET_PLANNED_TRAJECTORY_COORDINATES,
  SOCKET_PUCK_COLORS,
  SOCKET_PUCK_FIRST_CORNER,
  SOCKET_REAL_TRAJECTORY_COORDINATE,
  SOCKET_RESISTANCE,
  SOCKET_ROBOT_CONSUMPTION,
  SOCKET_TABLE_IMAGE,
  START_CYCLE,
} from './mutation-types';
import { defaultState, State } from './state';
import { Message } from '@/types/message';
import { Step } from '@/types/step';

export type Mutations<S = State> = {
  [START_CYCLE](state: S): void;
  [SOCKET_ROBOT_CONSUMPTION](state: S, data: string): void;
  [SOCKET_RESISTANCE](state: S, data: string): void;
  [SOCKET_PUCK_COLORS](state: S, data: string): void;
  [SOCKET_PUCK_FIRST_CORNER](state: S, data: string): void;
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: S, data: string): void;
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: S, data: string): void;
  [SOCKET_GRIP_STATE](state: S, data: string): void;
  [SOCKET_CURRENT_STEP](state: S, data: string): void;
  changeStep(State: S): void;
  changeGrip(State: S): void;
};

const toMessage = (data: string): Message => JSON.parse(data);

// TODO : Remove console logs, it's to test communication
export const mutations: MutationTree<State> & Mutations = {
  // TODO : Remove this
  changeStep(state: State) {
    state.currentStep++;
  },

  // TODO : Remove this (plus, it won't work!)
  changeGrip(state: State) {
    state.puckList.hasOneGripped = !state.puckList.hasOneGripped;
  },

  [START_CYCLE](state: State) {
    console.log('START_CYCLE : Sent!');
    state.currentStep = Step.CycleStarted;
    console.log(state);
  },
  [SOCKET_ROBOT_CONSUMPTION](state: State, data: string) {
    const message = toMessage(data);
    console.log('ROBOT_CONSUMPTION : Received!');
    console.log(message);
    state.cycleReady = true;
    state.robotConsumption =
      message.robotConsumption || defaultState.robotConsumption;
    console.log(state);
  },
  [SOCKET_TABLE_IMAGE](state: State, data: string) {
    const message = toMessage(data);
    console.log('TABLE_IMAGE : Received!');
    console.log(message);
    state.tableImage = message.tableImage || defaultState.tableImage;
    console.log(state);
  },
  [SOCKET_RESISTANCE](state: State, data: string) {
    const message = toMessage(data);
    console.log('RESISTANCE : Received!');
    console.log(message);
    state.resistance = message.resistance || defaultState.resistance;
    console.log(state);
  },
  [SOCKET_PUCK_COLORS](state: State, data: string) {
    const message = toMessage(data);
    console.log('PUCK_COLORS : Received!');
    console.log(message);
    state.puckList.colors = message.puckColors || defaultState.puckList.colors;
    console.log(state);
  },
  [SOCKET_PUCK_FIRST_CORNER](state: State, data: string) {
    const message = toMessage(data);
    console.log('PUCK_FIRST_CORNER : Received!');
    console.log(message);
    state.puckList.firstCorner =
      message.puckFirstCorner || defaultState.puckList.first.corner;
    console.log(state);
  },
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: State, data: string) {
    const message = toMessage(data);
    console.log('PLANNED_TRAJECTORY_COORDINATES : Received!');
    console.log(message);
    if (message.plannedTrajectoryCoordinates) {
      state.plannedTrajectory.push(...message.plannedTrajectoryCoordinates);
      state.currentPlannedTrajectory =
        message.plannedTrajectoryCoordinates ||
        defaultState.currentPlannedTrajectory;
    }
    console.log(state);
  },
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: State, data: string) {
    const message = toMessage(data);
    console.log('REAL_TRAJECTORY_COORDINATES : Received!');
    console.log(message);
    if (message.realTrajectoryCoordinate)
      state.realTrajectory.push(message.realTrajectoryCoordinate);
    console.log(state);
  },
  // TODO : Change puck first corner mutation
  [SOCKET_GRIP_STATE](state: State, data: string) {
    const message = toMessage(data);
    console.log('GRIP_STATE : Received!');
    console.log(message);
    state.puckList.hasOneGripped =
      message.puckInGrip || defaultState.puckList.hasOneGripped;
    console.log(state);
  },
  [SOCKET_CURRENT_STEP](state: State, data: string) {
    const message = toMessage(data);
    console.log('CURRENT_STEP : Received!');
    console.log(message);
    state.currentStep = message.currentStep || defaultState.currentStep;
    console.log(state);
  },
};
