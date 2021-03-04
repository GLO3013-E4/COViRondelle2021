import StepList from '@/components/cycles/StepList.vue';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { Step } from '@/types/step';
import { State } from '@/store/state';

describe('When mounting StepList component', () => {
  const wrapper = wrapWithVuetifyAndStore(StepList);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const state = {
    currentStep: Step.CycleNotStarted,
  } as State;

  describe('When mounting StepList', () => {
    const wrapper = wrapWithVuetifyAndStore(StepList, state);

    it('Should contains the right amount of steps', () => {
      const steps = wrapper.findAllComponents({ ref: 'step' });

      expect(steps.exists()).toBe(true);
      expect(steps).toHaveLength(15);
    });
  });
});
