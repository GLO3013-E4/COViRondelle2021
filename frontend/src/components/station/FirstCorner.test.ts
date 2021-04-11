import FirstCorner from '@/components/station/FirstCorner.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';
import { PuckListFactory } from '@/factories/PuckListFactory';

describe('When mounting FirstCorner component', () => {
  const wrapper = wrapWithVuetifyAndStore(FirstCorner);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const state = {
    puckList: PuckListFactory.make(),
  } as State;

  describe('When mounting FirstCorner', () => {
    const wrapper = wrapWithVuetifyAndStore(FirstCorner, state);

    it('Should contains the right letter of corner', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe(state.puckList.first.corner);
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
