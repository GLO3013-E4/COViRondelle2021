import Resistance from '@/components/station/Resistance.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { shallowMount, createLocalVue } from '@vue/test-utils';
import Vuex from 'vuex';
import { Color } from '@/types/color';

describe('When mounting Resistance component', () => {
  const wrapper = wrapWithVuetifyAndStore(Resistance);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
      resistance: 100000,
      puckColors: [Color.Red, Color.Blue, Color.Yellow],
    },
  });

  describe('When mounting Resistance', () => {
    const wrapper = shallowMount(Resistance, { store, localVue });

    it('Should contains the right resistanceValue', () => {
      const resistanceValue = wrapper.findComponent({ ref: 'resistanceValue' });

      expect(resistanceValue.exists()).toBe(true);
      expect(resistanceValue.text()).toBe('100000 Ω');
    });

    it('Should contains the right number of pucks', () => {
      const pucks = wrapper.findAllComponents({ ref: 'pucks' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(3);
    });
  });
});

describe('Given no state', () => {
  describe('When mounting Resistance', () => {
    const wrapper = wrapWithVuetifyAndStore(Resistance);

    it('Should not contain resistanceValue', () => {
      const resistanceValue = wrapper.findComponent({ ref: 'resistanceValue' });

      expect(resistanceValue.exists()).toBe(true);
      expect(resistanceValue.text()).toBe('Ω');
    });

    it('Should not contain pucks', () => {
      const pucks = wrapper.findAllComponents({ ref: 'pucks' });

      expect(pucks.exists()).toBe(false);
      expect(pucks).toHaveLength(0);
    });
  });
});
