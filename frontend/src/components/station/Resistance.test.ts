import Resistance from '@/components/station/Resistance.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { ColorFactory } from '@/factories/ColorFactory';

describe('When mounting Resistance component', () => {
  const wrapper = wrapWithVuetifyAndStore(Resistance);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const puckList = PuckListFactory.get();
  puckList.colors = ColorFactory.get(3);

  const state = {
    resistance: 100000,
    puckList,
  } as State;

  describe('When mounting Resistance', () => {
    const wrapper = wrapWithVuetifyAndStore(Resistance, state);

    it('Should contains the right resistanceValue', () => {
      const resistanceValue = wrapper.findComponent({ ref: 'resistanceValue' });

      expect(resistanceValue.exists()).toBe(true);
      expect(resistanceValue.text()).toBe(`${state.resistance} Ω`);
    });

    it('Should contains the right number of pucks', () => {
      const pucks = wrapper.findAllComponents({ ref: 'pucks' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(state.puckList.pucks.length);
    });
  });
});

describe('Given no state', () => {
  const wrapper = wrapWithVuetifyAndStore(Resistance);

  describe('When mounting Resistance', () => {
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
