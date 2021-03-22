import RemainingTime from '@/components/consumptions/RemainingTime.vue';
import { State } from '@/store/state';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';

describe('Given state', () => {
  const state = {
    robotConsumption: {
      wheel1: 0,
      wheel2: 0,
      wheel3: 0,
      wheel4: 0,
      total: 0,
      remainingTime: 60,
      batteryCharge: 0,
    },
  } as State;

  describe('When mounting RemainingTime', () => {
    const wrapper = wrapWithVuetifyAndStore(RemainingTime, state);

    it('Should contains the right time in hh:mm:ss', () => {
      const time = wrapper.findComponent({ ref: 'time' });

      expect(time.exists()).toBe(true);
      expect(time.text()).toBe('00:01:00');
    });
  });
});
