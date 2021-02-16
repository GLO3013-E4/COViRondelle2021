import ControlPanel from '@/components/station/ControlPanel.vue';
import { ColorFactory } from '@/factories/ColorFactory';
import Vue from 'vue';
import Vuex from 'vuex';
import { Corner } from '@/types/corner';
import { CornerFactory } from '@/factories/CornerFactory';

Vue.use(Vuex);

export default {
  title: 'components/station/ControlPanel',
  component: ControlPanel,
};

const Template = (args: any) => ({
  components: { ControlPanel },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<ControlPanel/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  puckFirstCorner: CornerFactory.get(),
  puckColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};

export const cornerA = Template.bind({}) as any;
cornerA.args = {
  puckFirstCorner: Corner.A,
  puckColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};

export const cornerB = Template.bind({}) as any;
cornerB.args = {
  puckFirstCorner: Corner.B,
  puckColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};

export const cornerC = Template.bind({}) as any;
cornerC.args = {
  puckFirstCorner: Corner.C,
  puckColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};

export const cornerD = Template.bind({}) as any;
cornerD.args = {
  puckFirstCorner: Corner.D,
  puckColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};
