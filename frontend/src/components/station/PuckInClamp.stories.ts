import PuckInClamp from '@/components/station/PuckInClamp.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';
import { defaultState } from '@/store/state';

Vue.use(Vuex);

export default {
  title: 'components/station/PuckInClamp',
  component: PuckInClamp,
};

const Template = (args: any) => ({
  components: { PuckInClamp },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<PuckInClamp />',
});

export const Default = Template.bind({}) as any;
Default.args = {
  puckInGrip: defaultState.puckInGrip,
};

export const ContainsPuck = Template.bind({}) as any;
ContainsPuck.args = {
  puckInGrip: true,
};

export const DontContainPuck = Template.bind({}) as any;
DontContainPuck.args = {
  puckInGrip: false,
};
