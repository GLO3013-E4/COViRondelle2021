import Resistance from '@/components/station/Resistance.vue';
import { ColorFactory } from '@/factories/ColorFactory';
import { Color } from '@/types/color';
import Vuex from 'vuex';
import Vue from 'vue';

Vue.use(Vuex);

export default {
  title: 'components/station/Resistance',
  component: Resistance,
};

const Template = (args: any, { argTypes }: any) => ({
  components: { Resistance },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<Resistance />',
});

export const Default = Template.bind({}) as any;
Default.args = {
  resistance: 100000,
  puckColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};

export const WhiteAndBlackPuck = Template.bind({}) as any;
WhiteAndBlackPuck.args = {
  resistance: 100000,
  puckColors: [Color.White, Color.Black, Color.Grey],
};

export const YellowPuck = Template.bind({}) as any;
YellowPuck.args = {
  resistance: 100000,
  puckColors: [Color.Yellow, ColorFactory.get(), ColorFactory.get()],
};
