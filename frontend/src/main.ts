import Vue from 'vue';
import VueRouter from 'vue-router';
import VueI18n from 'vue-i18n';
import App from './App.vue';
import vuetify from '@/plugins/vuetify';
import store from './store';
import { io } from 'socket.io-client';
import VueSocketIOExt from 'vue-socket.io-extended';
import { messages, defaultLocale } from './i18n';

Vue.config.productionTip = false;

// TODO : Change socket server URL
const STATION_URL = 'http://localhost:4000';
const socket = io(STATION_URL);
Vue.use(VueSocketIOExt, socket, { store });

Vue.use(VueRouter);
Vue.use(VueI18n);

const locale = window.location.pathname.replace(/^\/([^/]+).*/i, '$1');

const i18n = new VueI18n({
  locale: locale.trim().length && locale != '/' ? locale : defaultLocale,
  fallbackLocale: 'en',
  messages,
});

const routes = [{ path: '/', component: require('./views/Main') }];

const router = new VueRouter({
  base: locale.trim().length && locale != '/' ? '/' + locale : undefined,
  mode: 'history',
  routes: routes,
});

new Vue({
  vuetify,
  store, // TODO : Should store also be added here?
  router,
  i18n,
  render: (h) => h(App),
}).$mount('#app');
