import GripState from '@/components/station/GripState.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { PuckState } from '@/types/puckState';

const mockState = (puckStates: Array<PuckState>): State => {
  return {
    puckList: PuckListFactory.makeWithStates(puckStates),
  } as State;
};

describe('When mounting GripState component', () => {
  const wrapper = wrapWithVuetifyAndStore(GripState);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given puck in grip', () => {
  const state = mockState([
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]);

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
  const state = mockState([
    PuckState.RELEASED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting GripState', () => {
    const wrapper = wrapWithVuetifyAndStore(GripState, state);

    it('Should contain the right value', () => {
      const grip = wrapper.findComponent({ ref: 'gripState' });

      expect(grip.exists()).toBe(true);
      expect(grip.text()).toBe(wrapper.vm.$t('station.noPuck'));
    });
  });
});
