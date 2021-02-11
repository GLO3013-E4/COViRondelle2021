import Resistance from "@/components/station/Resistance.vue";
import { enumFactory } from 'node-factory';

const COLORS = ['yellow', 'brown', 'red', 'pink', 'orange','black', 'white', 'green', 'blue', 'purple','gray'];

const StateFactory = enumFactory<string>(COLORS);

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
