import Vue from 'vue';
import App from './App.vue';
import vuetify from '@/plugins/vuetify';
import store from './store';
import { io } from 'socket.io-client';
import VueSocketIOExt from 'vue-socket.io-extended';

Vue.config.productionTip = false;

const socket = io(process.env.VUE_APP_STATION_URL);
Vue.use(VueSocketIOExt, socket, { store });

new Vue({
  vuetify,
  store,
  render: (h) => h(App),
}).$mount('#app');
