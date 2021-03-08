import BatteryCharge from '@/components/consumptions/BatteryCharge.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { createLocalVue, shallowMount } from '@vue/test-utils';
import Vuex from 'vuex';

describe('When mounting BatteryCharge component', () => {
    const wrapper = wrapWithVuetifyAndStore(BatteryCharge);
  
    it('Should mount', () => {
      expect(wrapper.vm).toBeTruthy();
    });
  });

describe('Given state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
        robotConsumption: {
            batteryChargeLeft: 4.0,
            batteryRemainingTimeInSeconds: 0,
          },
    },
  });

  describe('When mounting BatteryCharge', () => {
    const wrapper = shallowMount(BatteryCharge, { store, localVue });

    it('Should contains the right letter of corner', () => {
      const batteryCharge = wrapper.findComponent({ ref: 'batteryCharge' });

      expect(batteryCharge.exists()).toBe(true);
      expect(batteryCharge.text()).toBe("4.000 Ah");
    });
  });
});
