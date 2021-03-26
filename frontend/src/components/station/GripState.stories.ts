import GripState from '@/components/station/GripState.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/station/GripState',
  component: GripState,
};

const Template = (args: any) => ({
  components: { GripState },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<GripState />',
});

export const WithPuckInGrip = Template.bind({}) as any;
WithPuckInGrip.args = {
  puckInGrip: true,
};

export const WithoutPuckInGrip = Template.bind({}) as any;
WithoutPuckInGrip.args = {
  puckInGrip: false,
};
