import Chronometer from '@/components/cycles/Chronometer.vue';
import { Step } from '@/types/step';
import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import { shallowMount, createLocalVue } from '@vue/test-utils';
import Vuex from 'vuex';

const mockStore = (cycleReady: boolean, currentStep: Step) =>
  new Vuex.Store({
    state: {
      cycleReady,
      currentStep,
    },
  });

describe('When mounting Chronometer component', () => {
  const wrapper = wrapWithVuetifyAndStore(Chronometer);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given no puck released yet', () => {
  const localVue = createLocalVue();
  localVue.use(Vuex);

  const store = mockStore(true, Step.CycleNotStarted);

  describe('When mounting PuckDeposit', () => {
    const wrapper = shallowMount(Chronometer, { store, localVue });

    it('Should have button and time', () => {
      const button = wrapper.findAllComponents({ ref: 'button' });
      const time = wrapper.findComponent({ ref: 'time' });

      expect(button.exists()).toBe(true);
      expect(time.exists()).toBe(true);
      expect(time.text()).toMatch('0 : 0.0');
    });
    });
});
