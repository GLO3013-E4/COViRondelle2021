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

  export const WhiteAndBlackPuck = Template.bind({}) as any;
  WhiteAndBlackPuck.args = {
    resistanceValue: 100000,
    pucksColors: ["white", "black", "grey"],
  };  

  export const YellowPuck = Template.bind({}) as any;
  YellowPuck.args = {
    resistanceValue: 100000,
    pucksColors: ["yellow", StateFactory.get(), StateFactory.get()],
  }; 
