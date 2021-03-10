import FirstCorner from '@/components/station/FirstCorner.vue';
import { ColorFactory } from '@/factories/ColorFactory';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { CornerFactory } from '@/factories/CornerFactory';
import { State } from '@/store/state';

describe('When mounting FirstCorner component', () => {
  const wrapper = wrapWithVuetifyAndStore(FirstCorner);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const state = {
    puckFirstCorner: CornerFactory.get(),
    puckColors: ColorFactory.get(1),
  } as State;

  describe('When mounting FirstCorner', () => {
    const wrapper = wrapWithVuetifyAndStore(FirstCorner, state);

    it('Should contains the right letter of corner', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe(state.puckFirstCorner);
    });
  });
});

describe('Given no state', () => {
  const wrapper = wrapWithVuetifyAndStore(FirstCorner);

  describe('When mounting FirstCorner component without props', () => {
    it('Should not contains resistanceValue', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe('');
    });
  });
});
