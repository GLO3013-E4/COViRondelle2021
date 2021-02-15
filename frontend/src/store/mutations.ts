import { MutationTree } from 'vuex/types';
import {
  SOCKET_ROBOT_CONSUMPTION,
  SOCKET_RESISTANCE_AND_PUCK_COLORS,
  SOCKET_PUCK_FIRST_CORNER,
  SOCKET_PLANNED_TRAJECTORY_COORDINATE,
  SOCKET_REAL_TRAJECTORY_COORDINATE,
} from './mutation-types';
import { defaultState, State } from './state';
import { Message } from '@/types/message';

export type Mutations<S = State> = {
  [SOCKET_ROBOT_CONSUMPTION](state: S, message: Message): void;
  [SOCKET_RESISTANCE_AND_PUCK_COLORS](state: S, message: Message): void;
  [SOCKET_PUCK_FIRST_CORNER](state: S, message: Message): void;
  [SOCKET_PLANNED_TRAJECTORY_COORDINATE](state: S, message: Message): void;
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: S, message: Message): void;
};

export const mutations: MutationTree<State> & Mutations = {
  [SOCKET_ROBOT_CONSUMPTION](state: State, message: Message) {
    // TODO : Implement get robot consumption from state in associated component
    state.robotConsumption =
      message.robotConsumption || defaultState.robotConsumption;
  },
  [SOCKET_RESISTANCE_AND_PUCK_COLORS](state: State, message: Message) {
    // TODO : Implement get resistance from state in associated component
    // TODO : Implement get puck colors from state in associated component
    state.resistance = message.resistance || defaultState.resistance;
    state.puckColors = message.puckColors || defaultState.puckColors;
  },
  [SOCKET_PUCK_FIRST_CORNER](state: State, message: Message) {
    // TODO : Implement get puck first corner from state in associated component
    state.puckFirstCorner =
      message.puckFirstCorner || defaultState.puckFirstCorner;
  },
  [SOCKET_PLANNED_TRAJECTORY_COORDINATE](state: State, message: Message) {
    // TODO : Implement get planned trajectory coordinate from state in associated component
    if (message.plannedTrajectoryCoordinate)
      state.plannedTrajectory.push(message.plannedTrajectoryCoordinate);
  },
  [SOCKET_REAL_TRAJECTORY_COORDINATE](state: State, message: Message) {
    // TODO : Implement get real trajectory coordinate from state in associated component
    if (message.realTrajectoryCoordinate)
      state.realTrajectory.push(message.realTrajectoryCoordinate);
  },
};
