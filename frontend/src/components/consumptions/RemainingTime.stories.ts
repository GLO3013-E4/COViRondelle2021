import RemainingTime from '@/components/consumptions/RemainingTime.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/consumptions/RemainingTime',
  component: RemainingTime,
};

const Template = (args: any) => ({
  components: { RemainingTime },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<RemainingTime />',
});

export const HoursLeft = Template.bind({}) as any;
HoursLeft.args = {
  robotConsumption: {
    batteryChargeLeft: 0,
    batteryRemainingTimeInSeconds: 3 * 60 * 60,
  },
};

export const MinutesLeft = Template.bind({}) as any;
MinutesLeft.args = {
  robotConsumption: {
    batteryChargeLeft: 0,
    batteryRemainingTimeInSeconds: 60 * 40,
  },
};

export const SecondsLeft = Template.bind({}) as any;
SecondsLeft.args = {
  robotConsumption: { batteryChargeLeft: 0, batteryRemainingTimeInSeconds: 34 },
};

export const aWeirdNumber = Template.bind({}) as any;
aWeirdNumber.args = {
  robotConsumption: {
    batteryChargeLeft: 0,
    batteryRemainingTimeInSeconds: 1234,
  },
};
