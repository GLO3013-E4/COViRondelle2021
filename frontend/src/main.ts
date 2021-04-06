import Vue from 'vue';
import VueRouter from 'vue-router';
import VueI18n from 'vue-i18n';
import App from './App.vue';
import vuetify from '@/plugins/vuetify';
import store from './store';
import { io } from 'socket.io-client';
import VueSocketIOExt from 'vue-socket.io-extended';
import { messages, defaultLocale } from './i18n';
import { library } from '@fortawesome/fontawesome-svg-core';
 import { fas } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome';
import { fab } from '@fortawesome/free-brands-svg-icons';

library.add(fas, fab);

Vue.component('font-awesome-icon', FontAwesomeIcon);

Vue.config.productionTip = false;

const socket = io(process.env.VUE_APP_STATION_URL);
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
  store,
  router,
  i18n,
  render: (h) => h(App),
}).$mount('#app');
