import RemainingTime from '@/components/consumptions/RemainingTime.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { shallowMount, createLocalVue } from '@vue/test-utils';
import Vuex from 'vuex';

describe('Given state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
        robotConsumption: {
            batteryChargeLeft: 0,
            batteryRemainingTimeInSeconds: 60,
          },
    },
  });

  describe('When mounting RemainingTime', () => {
    const wrapper = shallowMount(RemainingTime, { store, localVue });

    it('Should contains the right time in hh:mm:ss', () => {
      const time = wrapper.findComponent({ ref: 'time' });

      expect(time.exists()).toBe(true);
      expect(time.text()).toBe('00:01:00');
    });
  });
});