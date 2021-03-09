import { MutationTree } from 'vuex/types';
import {
  START_CYCLE,
  SOCKET_ROBOT_CONSUMPTION,
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
  [SOCKET_ROBOT_CONSUMPTION](state: S, message: Message): void;
  [SOCKET_TABLE_IMAGE](state: S, message: Message): void;
  [SOCKET_RESISTANCE](state: S, message: Message): void;
  [SOCKET_PUCK_COLORS](state: S, message: Message): void;
  [SOCKET_PUCK_FIRST_CORNER](state: S, message: Message): void;
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: S, message: Message): void;
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: S, message: Message): void;
  [SOCKET_GRIP_STATE](state: S, message: Message): void;
  [SOCKET_CURRENT_STEP](state: S, message: Message): void;
};

export const mutations: MutationTree<State> & Mutations = {
  [START_CYCLE](state: State) {
    state.currentStep = Step.CycleStarted;
  },
  [SOCKET_CYCLE_READY](state: State) {
    // TODO : Implement get cycle ready from state in associated component
    console.log('Frontend : Received cycle ready!'); // TODO : Remove console log
    state.cycleReady = true;
  },
  [SOCKET_ROBOT_CONSUMPTION](state: State, message: Message) {
    // TODO : Implement get robot consumption from state in associated component
    state.robotConsumption =
      message.robotConsumption || defaultState.robotConsumption;
  },
  [SOCKET_TABLE_IMAGE](state: State, message: Message) {
    // TODO : Implement get table image from state in associated component
    state.tableImage = message.tableImage || defaultState.tableImage;
  },
  [SOCKET_RESISTANCE](state: State, message: Message) {
    state.resistance = message.resistance || defaultState.resistance;
  },
  [SOCKET_PUCK_COLORS](state: State, message: Message) {
    state.puckColors = message.puckColors || defaultState.puckColors;
  },
  [SOCKET_PUCK_FIRST_CORNER](state: State, message: Message) {
    state.puckFirstCorner =
      message.puckFirstCorner || defaultState.puckFirstCorner;
  },
  [SOCKET_PLANNED_TRAJECTORY_COORDINATES](state: State, message: Message) {
    // TODO : Implement get planned trajectory coordinate from state in associated component
    if (message.plannedTrajectoryCoordinates)
      state.plannedTrajectory.push(...message.plannedTrajectoryCoordinates);
  },
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: State, message: Message) {
    // TODO : Implement get real trajectory coordinate from state in associated component
    console.log(message);
    if (message.realTrajectoryCoordinate)
      state.realTrajectory.push(message.realTrajectoryCoordinate);
  },
  [SOCKET_GRIP_STATE](state: State, message: Message) {
    // TODO : Implement get puck grip state from state in associated component
    state.puckInGrip = message.puckInGrip || defaultState.puckInGrip;
  },
  [SOCKET_CURRENT_STEP](state: State, message: Message) {
    // TODO : Implement get current step from state in associated component
    state.currentStep = message.currentStep || defaultState.currentStep;
  },
};
