import RobotConsumptionInfo from '@/components/consumptions/RobotConsumptionInfo.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';
import { RobotConsumption } from '@/types/robotConsumption';

describe('Given state', () => {
  const state = {
    robotConsumption: {
        wheel1: 0,
        wheel2: 0,
        wheel3: 0,
        wheel4: 0,
        servoMotor: 0,
        total: 0,
    } as RobotConsumption,
  } as State;

  describe('When mounting RobotConsumptionInfo', () => {
    const wrapper = wrapWithVuetifyAndStore(RobotConsumptionInfo, state);

    it('Should contains the right number of consumptions details', () => {
        const consumptions = wrapper.findComponent({ref:"consumptions"})
        const everyConsumption = consumptions.findAll("h5");

      expect(everyConsumption.exists()).toBe(true);
      expect(everyConsumption).toHaveLength(6);
    });
  });
});