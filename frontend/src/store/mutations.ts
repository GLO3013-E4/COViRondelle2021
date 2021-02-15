import { MutationTree } from 'vuex/types';
import {
  SOCKET_ONOPEN,
  SOCKET_ONCLOSE,
  SOCKET_ONERROR,
  SOCKET_ONMESSAGE,
  SOCKET_RECONNECT,
  SOCKET_RECONNECT_ERROR,
} from './mutation-types';
import { State } from './state';

export type Mutations<S = State> = {
  [SOCKET_ONOPEN](state: S, _event: any): void;
  [SOCKET_ONCLOSE](state: S, _event: any): void;
  [SOCKET_ONERROR](state: S, event: any): void;
  // TODO : Define message type
  [SOCKET_ONMESSAGE](state: S, message: any): void;
  [SOCKET_RECONNECT](state: S, count: number): void;
  [SOCKET_RECONNECT_ERROR](state: S): void;
};

export const mutations: MutationTree<State> & Mutations = {
  [SOCKET_ONOPEN](state: State, _event: any) {
    state.socket.isConnected = true;
  },
  [SOCKET_ONCLOSE](state: State, _event: any) {
    state.socket.isConnected = false;
  },
  [SOCKET_ONERROR](state: State, event: any) {
    console.error(state, event);
  },
  [SOCKET_ONMESSAGE](state: State, message: any) {
    // TODO : Handle each message type
    console.info(state, message);
  },
  [SOCKET_RECONNECT](state: State, count: number) {
    // TODO : Handle reconnect somehow
    console.info(state, count);
  },
  [SOCKET_RECONNECT_ERROR](state: State) {
    state.socket.reconnectError = true;
  },
};
