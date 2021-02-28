import StepList from '@/components/cycles/StepList.vue';
import { createLocalVue, shallowMount } from '@vue/test-utils';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import Vuex from 'vuex';
import { Step } from '@/types/step';

describe('When mounting StepList component', () => {
  const wrapper = wrapWithVuetifyAndStore(StepList);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = new Vuex.Store({
    state: {
      currentStep: Step.CycleNotStarted
    },
  });

  describe('When mounting StepList', () => {
    const wrapper = shallowMount(StepList, { store, localVue });

    it('Should contains the right amount of steps', () => {
        const steps = wrapper.findAllComponents({ ref: 'step' });
        
        expect(steps.exists()).toBe(true);
        expect(steps).toHaveLength(15);
    });
  });
});
