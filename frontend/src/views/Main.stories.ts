import Main from '@/views/Main.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'views/Main',
  component: Main,
};

export const Default = () => ({
  components: { Main },
  store: new Vuex.Store({}),
  template: `<Main />`,
});
