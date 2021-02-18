import StationInformation from '@/components/station/StationInformation.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';

Vue.use(Vuex);

export default {
  title: 'components/station/StationInformation',
  component: StationInformation,
};

export const Default = () => ({
  components: { StationInformation },
  store: new Vuex.Store({
    state: StateFactory.make(),
  }),
  template: `<station-information/>`,
});
