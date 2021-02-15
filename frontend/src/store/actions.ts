import Vue from 'vue';
import { SEND_START_CYCLE } from './action-types';

export const actions = {
  [SEND_START_CYCLE]() {
    // TODO : Implement correctly sending messages
    // TODO : Use message type enum
    Vue.prototype.$socket.send('Start cycle!');
  },
};
