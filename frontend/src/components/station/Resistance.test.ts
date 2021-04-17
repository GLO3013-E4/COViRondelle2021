import Resistance from '@/components/station/Resistance.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { Color } from '@/types/color';

describe('When mounting Resistance component', () => {
  const wrapper = wrapWithVuetifyAndStore(Resistance);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const state = {
    resistance: 100000,
    puckList: PuckListFactory.make(),
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
      expect(pucks).toHaveLength(
        state.puckList.pucks.filter((puck) => puck.color !== Color.Unset).length
      );
    });
  });
});

describe('Given no state', () => {
  const wrapper = wrapWithVuetifyAndStore(Resistance);

  describe('When mounting Resistance', () => {
    it('Should not contain resistanceValue', () => {
      const resistanceValue = wrapper.findComponent({ ref: 'resistanceValue' });

      expect(resistanceValue.exists()).toBe(true);
      expect(resistanceValue.text()).toBe('0 Ω');
    });

    it('Should not contain pucks', () => {
      const pucks = wrapper.findAllComponents({ ref: 'pucks' });

      expect(pucks.exists()).toBe(false);
      expect(pucks).toHaveLength(0);
    });
  });
});
