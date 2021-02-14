import ControlPanelResult from '@/classes/ControlPanelResult';
import ControlPanel from '@/components/station/ControlPanel.vue';
import { ColorFactory } from '@/factories/ColorFactory';
import { ControlPanelResultFactory } from '@/factories/ControlPanelResult';

export default {
  title: 'components/station/ControlPanel',
  component: ControlPanel,
};

const Template = (args: any, { argTypes }: any) => ({
  components: { ControlPanel },
  props: Object.keys(argTypes),
  template: '<ControlPanel v-bind="$props"/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
    controlPanelResult: ControlPanelResultFactory.get(),
    colorFirstPuck: ColorFactory.get(),
};

export const noData = Template.bind({}) as any;
noData.args = {
    controlPanelResult: new ControlPanelResult(""),
    colorFirstPuck: ColorFactory.get(),
};

export const coinA = Template.bind({}) as any;
coinA.args = {
    controlPanelResult: new ControlPanelResult("A"),
    colorFirstPuck: ColorFactory.get(),
};

export const coinB = Template.bind({}) as any;
coinB.args = {
    controlPanelResult: new ControlPanelResult("B"),
    colorFirstPuck: ColorFactory.get(),
};

export const coinC = Template.bind({}) as any;
coinC.args = {
    controlPanelResult: new ControlPanelResult("C"),
    colorFirstPuck: ColorFactory.get(),
};

export const coinD = Template.bind({}) as any;
coinD.args = {
    controlPanelResult: new ControlPanelResult("D"),
    colorFirstPuck: ColorFactory.get(),
};
