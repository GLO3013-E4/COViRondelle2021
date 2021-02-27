import PuckDeposited from '@/components/station/PuckDeposited.vue';
import { ColorFactory } from '@/factories/ColorFactory';
import { Color } from '@/types/color';
import { Step } from '@/types/step';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/station/PuckDeposited',
  component: PuckDeposited,
};

const Template = (args: any) => ({
  components: { PuckDeposited },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<PuckDeposited v-bind="$props"/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: false,
  currentStep: Step.CycleNotStarted,
};

export const FirstPuckReleased = Template.bind({}) as any;
FirstPuckReleased.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: false,
  currentStep: Step.ToFirstCornerAndReleaseFirstPuck,
};

export const FirstPuckNotReleasedYet = Template.bind({}) as any;
FirstPuckNotReleasedYet.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: true,
  currentStep: Step.ToFirstCornerAndReleaseFirstPuck,
};

export const SecondPuckReleased = Template.bind({}) as any;
SecondPuckReleased.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: false,
  currentStep: Step.ToSecondCornerAndReleaseSecondPuck,
};

export const SecondPuckNotReleasedYet = Template.bind({}) as any;
SecondPuckNotReleasedYet.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: true,
  currentStep: Step.ToSecondCornerAndReleaseSecondPuck,
};

export const ThirdPuckReleased = Template.bind({}) as any;
ThirdPuckReleased.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: false,
  currentStep: Step.ToThirdCornerAndReleaseThirdPuck,
};

export const ThirdPuckNotReleasedYet = Template.bind({}) as any;
ThirdPuckNotReleasedYet.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: true,
  currentStep: Step.ToThirdCornerAndReleaseThirdPuck,
};

export const StepAfterLastRelease = Template.bind({}) as any;
StepAfterLastRelease.args = {
  puckColors: ColorFactory.get(3),
  puckInGrip: false,
  currentStep: Step.CycleEndedAndRedLedOn,
};
