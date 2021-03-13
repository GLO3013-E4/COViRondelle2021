import BatteryCharge from '@/components/consumptions/BatteryCharge.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';

describe('Given state', () => {
  const state = {
    robotConsumption: {
      batteryChargeLeft: 4.0,
      batteryRemainingTimeInSeconds: 0,
    },
  } as State;

  describe('When mounting BatteryCharge', () => {
    const wrapper = wrapWithVuetifyAndStore(BatteryCharge, state);

    it('Should contains the right letter of corner', () => {
      const batteryCharge = wrapper.findComponent({ ref: 'batteryCharge' });

      expect(batteryCharge.exists()).toBe(true);
      expect(batteryCharge.text()).toBe("4.000 Ah");
    });
  });
});
