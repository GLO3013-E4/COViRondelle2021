import { MutationTree } from 'vuex/types';
import { SOCKET_RESISTANCE, SOCKET_ROBOT_CONSUMPTION } from './mutation-types';
import { defaultState, State } from './state';
import { Message } from '@/types/message';

export type Mutations<S = State> = {
  [SOCKET_RESISTANCE](state: S, message: Message): void;
  [SOCKET_ROBOT_CONSUMPTION](state: S, message: Message): void;
};

export const mutations: MutationTree<State> & Mutations = {
  [SOCKET_RESISTANCE](state: State, message: Message) {
    // TODO : Implement get resistance from state in associated component
    state.resistance = message.resistance || defaultState.resistance;
  },
  [SOCKET_ROBOT_CONSUMPTION](state: State, message: Message) {
    // TODO : Implement get robot consumption from state in associated component
    state.robotConsumption =
      message.robotConsumption || defaultState.robotConsumption;
  },
};
