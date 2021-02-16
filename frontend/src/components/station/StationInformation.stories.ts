import StationInformation from '@/components/station/StationInformation.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/station/StationInformation',
  component: StationInformation,
};

export const Default = () => ({
  components: { StationInformation },
  store: new Vuex.Store({}),
  template: `<station-information/>`,
});
