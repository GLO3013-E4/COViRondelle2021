import GripState from '@/components/station/GripState.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';

describe('When mounting GripState component', () => {
  const wrapper = wrapWithVuetifyAndStore(GripState);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given puck in grip', () => {
  const state = {
    puckInGrip: true,
  } as State;

  describe('When mounting GripState', () => {
    const wrapper = wrapWithVuetifyAndStore(GripState, state);

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(grip.text()).toBe(wrapper.vm.$t('station.puckInGrip'));
    });
  });
});

describe('Given puck released', () => {
  const state = {
    puckInGrip: false,
  } as State;

  describe('When mounting GripState', () => {
    const wrapper = wrapWithVuetifyAndStore(GripState, state);

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(grip.text()).toBe(wrapper.vm.$t('station.noPuck'));
    });
  });
});
