import Vue from 'vue';
import App from './App.vue';
import vuetify from '@/plugins/vuetify';
import store from './store';
import { io } from 'socket.io-client';
import VueSocketIOExt from 'vue-socket.io-extended';

Vue.config.productionTip = false;

// TODO : Change socket server URL
const STATION_URL = 'http://localhost:4000'
const socket = io(STATION_URL);
Vue.use(VueSocketIOExt, socket, { store });

new Vue({
  vuetify,
  store, // TODO : Should store also be added here?
  render: (h) => h(App),
}).$mount('#app');
