import Chronometer from '@/components/cycles/Chronometer.vue';
import { Step } from '@/types/step';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/cycles/Chronometer',
  component: Chronometer,
};

const Template = (args: any) => ({
  components: { Chronometer },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<Chronometer />',
});

export const WhenCycleNotStarted = Template.bind({}) as any;
WhenCycleNotStarted.args = {
  currentStep: Step.CycleNotStarted,
};

export const WhenCycleEnds = Template.bind({}) as any;
WhenCycleEnds.args = {
  currentStep: Step.CycleEndedAndRedLedOn,
};