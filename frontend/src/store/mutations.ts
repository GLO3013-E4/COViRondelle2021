import { MutationTree } from 'vuex/types';
import { SOCKET_RESISTANCE } from './mutation-types';
import { State } from './state';
import { Message } from '@/types/message';

export type Mutations<S = State> = {
  [SOCKET_RESISTANCE](state: S, message: Message): void;
};

export const mutations: MutationTree<State> & Mutations = {
  [SOCKET_RESISTANCE](state: State, message: Message) {
    // TODO : Implement get resistance from state in associated component
    state.resistance = message.resistance || 0;
  },
};
