import StartButton from '@/components/cycles/StartButton.vue';
import Vuex from 'vuex';
import { Step } from '@/types/step';

export default {
  title: 'components/cycles/StartButton',
  component: StartButton,
};

const Template = (args: any) => ({
  components: { StartButton },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<StartButton  v-bind="$props" />',
});

export const WithCycleNotReady = Template.bind({}) as any;
WithCycleNotReady.args = {
  cycleReady: false,
  currentStep: Step.CycleNotStarted,
};

export const WithCycleReady = Template.bind({}) as any;
WithCycleReady.args = {
  cycleReady: true,
  currentStep: Step.CycleReadyInWaitingMode,
};
