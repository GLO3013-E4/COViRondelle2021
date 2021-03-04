import PuckInGrip from '@/components/station/PuckInGrip.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/station/PuckInGrip',
  component: PuckInGrip,
};

const Template = (args: any) => ({
  components: { PuckInGrip },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<PuckInGrip />',
});

export const WithPuckInGrip = Template.bind({}) as any;
WithPuckInGrip.args = {
  puckInGrip: true,
};

export const WithoutPuckInGrip = Template.bind({}) as any;
WithoutPuckInGrip.args = {
  puckInGrip: false,
};
