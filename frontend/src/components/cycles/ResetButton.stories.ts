import ResetButton from '@/components/cycles/ResetButton.vue';
import Vuex from 'vuex';
import { Step } from '@/types/step';

export default {
  title: 'components/cycles/ResetButton',
  component: ResetButton,
};

const Template = (args: any) => ({
  components: { ResetButton },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<ResetButton  v-bind="$props" />',
});

export const CycleNotEnded = Template.bind({}) as any;
CycleNotEnded.args = {
  currentStep: Step.CycleStarted,
};

export const CycleEnded = Template.bind({}) as any;
CycleEnded.args = {
  currentStep: Step.CycleEndedAndRedLedOn,
};
