import Chronometer from '@/components/cycles/Chronometer.vue';
import { Step } from '@/types/step';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { State } from '@/store/state';

describe('When mounting Chronometer component', () => {
  const wrapper = wrapWithVuetifyAndStore(Chronometer);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given no puck released yet', () => {
  const state = {
    cycleReady: true,
    currentStep: Step.CycleNotStarted,
  } as State;

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapWithVuetifyAndStore(Chronometer, state);

    it('Should have button and time', () => {
      const button = wrapper.findAllComponents({ ref: 'button' });
      const time = wrapper.findComponent({ ref: 'time' });

      expect(button.exists()).toBe(true);
      expect(time.exists()).toBe(true);
      expect(time.text()).toMatch('0 : 0.0');
    });
  });
});
