import Resistance from "@/components/station/Resistance.vue";
import { StateFactory } from "../../Factories/resistanceFactory";

export default{
    title: 'components/station/Resistance',
    component: Resistance
};
  
const Template = (args: any, { argTypes }: any) => ({
    components: { Resistance },
    props: Object.keys(argTypes),
    template: '<Resistance v-bind="$props"/>',
  });
  
  export const Default = Template.bind({}) as any;
  Default.args = {
    resistanceValue: 100000,
    pucksColors: [StateFactory.get(), StateFactory.get(), StateFactory.get()],
  };

  export const WhiteAndBlackResistance = Template.bind({}) as any;
  WhiteAndBlackResistance.args = {
    resistanceValue: 100000,
    pucksColors: ["white", "black", "grey"],
  };  
