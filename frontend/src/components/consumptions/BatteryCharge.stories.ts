import BatteryCharge from '@/components/consumptions/BatteryCharge.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/consumptions/BatteryCharge',
  component: BatteryCharge,
};

const Template = (args: any) => ({
  components: { BatteryCharge },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<BatteryCharge />',
});

export const AllChargeLeft = Template.bind({}) as any;
AllChargeLeft.args = {
  batteryConsumption: { batteryChargeLeft: 8, batteryRemainingTimeInSeconds: 0 },
};

export const HalfChargeLeft = Template.bind({}) as any;
HalfChargeLeft.args = {
  batteryConsumption: {
    batteryChargeLeft: 4.0,
    batteryRemainingTimeInSeconds: 0,
  },
};

export const ThirdChargeLeft = Template.bind({}) as any;
ThirdChargeLeft.args = {
  batteryConsumption: { batteryChargeLeft: 2, batteryRemainingTimeInSeconds: 0 },
};

export const NoChargeLeft = Template.bind({}) as any;
NoChargeLeft.args = {
  batteryConsumption: { batteryChargeLeft: 0, batteryRemainingTimeInSeconds: 0 },
};

export const ChargeWith3Decimals = Template.bind({}) as any;
ChargeWith3Decimals.args = {
  batteryConsumption: {
    batteryChargeLeft: 2.455,
    batteryRemainingTimeInSeconds: 0,
  },
};

export const ChargeWith2Decimals = Template.bind({}) as any;
ChargeWith2Decimals.args = {
  batteryConsumption: {
    batteryChargeLeft: 5.04,
    batteryRemainingTimeInSeconds: 0,
  },
};
