import FirstCorner from '@/components/station/FirstCorner.vue';
import { ColorFactory } from '@/factories/ColorFactory';
import Vue from 'vue';
import Vuex from 'vuex';
import { Corner } from '@/types/corner';
import { CornerFactory } from '@/factories/CornerFactory';

Vue.use(Vuex);

export default {
  title: 'components/station/FirstCorner',
  component: FirstCorner,
};

const Template = (args: any) => ({
  components: { ControlPanel: FirstCorner },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<ControlPanel/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  puckFirstCorner: CornerFactory.get(),
  puckColors: ColorFactory.get(1),
};

export const cornerA = Template.bind({}) as any;
cornerA.args = {
  puckFirstCorner: Corner.A,
  puckColors: ColorFactory.get(1),
};

export const cornerB = Template.bind({}) as any;
cornerB.args = {
  puckFirstCorner: Corner.B,
  puckColors: ColorFactory.get(1),
};

export const cornerC = Template.bind({}) as any;
cornerC.args = {
  puckFirstCorner: Corner.C,
  puckColors: ColorFactory.get(1),
};

export const cornerD = Template.bind({}) as any;
cornerD.args = {
  puckFirstCorner: Corner.D,
  puckColors: ColorFactory.get(1),
};
