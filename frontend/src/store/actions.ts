import Vue from 'vue';
import { EMIT_SOCKET_START_CYCLE } from './action-types';

export const actions = {
  // TODO : Implement emitSocketStartCycle in associated button
  [EMIT_SOCKET_START_CYCLE]() {
    Vue.prototype.$socket.client.emit('start_cycle');
    // TODO : Start cycle in chrono component
  },
};
