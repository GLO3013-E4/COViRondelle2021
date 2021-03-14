import RobotConsumptionInfo from '@/components/consumptions/RobotConsumptionInfo.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/consumptions/RobotConsumptionInfo',
  component: RobotConsumptionInfo,
};

const Template = (args: any) => ({
  components: { RobotConsumptionInfo },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<RobotConsumptionInfo />',
});

export const anyData = Template.bind({}) as any;
anyData.args = {
  robotConsumption: { wheel1: 800, wheel2: 504, wheel3: 0, wheel4: 0, servoMotor:0, total:0},
};