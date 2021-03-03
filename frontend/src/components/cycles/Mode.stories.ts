import Mode from '@/components/cycles/Mode.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/cycles/Mode',
  component: Mode,
};

const Template = (args: any) => ({
  components: { Mode },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<Mode />',
});

export const cycleReadyInWaitingMode = Template.bind({}) as any;
cycleReadyInWaitingMode.args = {
  cycleReady: true,
  cycleStarted: false
};

export const cycleStarted = Template.bind({}) as any;
cycleStarted.args = {
    cycleReady: true,
    cycleStarted: true
};

export const cycleNotStartedAndRobotStillBooting = Template.bind({}) as any;
cycleNotStartedAndRobotStillBooting.args = {
    cycleReady: false,
    cycleStarted: false
};
