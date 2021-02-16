import ControlPanel from '@/components/station/ControlPanel.vue';
import { createLocalVue, shallowMount } from '@vue/test-utils';
import { ColorFactory } from '@/factories/ColorFactory';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import Vuex from 'vuex';
import { CornerFactory } from '@/factories/CornerFactory';

describe('When mounting ControlPanel component', () => {
  const wrapper = wrapWithVuetifyAndStore(ControlPanel);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
      puckFirstCorner: CornerFactory.get(),
      puckColors: ColorFactory.get(1),
    },
  });

  describe('When mounting ControlPanel', () => {
    const wrapper = shallowMount(ControlPanel, { store, localVue });

    it('Should contains the right letter of corner', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe(store.state.puckFirstCorner.toString());
    });
  });
});

describe('Given no state', () => {
  const wrapper = wrapWithVuetifyAndStore(ControlPanel);

  describe('When mounting ControlPanel component without props', () => {
    it('Should not contains resistanceValue', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe('');
    });
  });
});
