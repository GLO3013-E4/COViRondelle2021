import Resistance from "@/components/station/Resistance.vue";
import { ColorFactory } from "@/factories/ColorFactory";

export default {
  title: "components/station/Resistance",
  component: Resistance,
};

const Template = (args: any, { argTypes }: any) => ({
  components: { Resistance },
  props: Object.keys(argTypes),
  template: '<Resistance v-bind="$props"/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  resistanceValue: 100000,
  pucksColors: [ColorFactory.get(), ColorFactory.get(), ColorFactory.get()],
};

export const WhiteAndBlackPuck = Template.bind({}) as any;
WhiteAndBlackPuck.args = {
  resistanceValue: 100000,
  pucksColors: ["white", "black", "grey"],
};

export const YellowPuck = Template.bind({}) as any;
YellowPuck.args = {
  resistanceValue: 100000,
  pucksColors: ["yellow", ColorFactory.get(), ColorFactory.get()],
};
