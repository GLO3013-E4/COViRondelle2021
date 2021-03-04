import Vue from 'vue';
import { EMIT_SOCKET_START_CYCLE } from './action-types';
import { START_CYCLE } from '@/store/mutation-types';

export const actions = {
  // TODO : Implement emitSocketStartCycle in associated button
  [EMIT_SOCKET_START_CYCLE]({ commit }: any) {
    console.log('Frontend : Sending start cycle!'); // TODO : Remove console log
    Vue.prototype.$socket.client.emit('start_cycle');
    commit(START_CYCLE);
  },
};
